/**
 * @file   coro_lift_exekutor.cpp
 * @author Chittaranjan S Srinivas
 * 
 * @brief  Provides the exekutor interface for enter/exit lift.
 *     
 * Copyright (C) 2015  Chittaranjan Srinivas Swaminathan
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 *
 * 
 */

#include <exekutor/coro_lift_exekutor.hpp>
#include <cam_interface/cam_interface.h>

namespace exekutor {

	CoroLiftExekutor::CoroLiftExekutor(std::string robot_name, std::string action_name) :
		ActionExekutor(robot_name, action_name),
		liftDoorOpen(false),
		liftDoorOpenPrev(false),
		mb_client_("move_base", true)
	{
	
	}

	void CoroLiftExekutor::laserCallback(const sensor_msgs::LaserScanConstPtr& scan) {

		std::vector<float> angle_data(scan->ranges.size(), 0.0);
		std::vector<float> laser_data = scan->ranges;
		//std::cout<<"\nlaser_data.size(): "<<data_ptr_->ranges.size()<<"\n";

		for(int i = 0; i<angle_data.size(); i++)
			angle_data[i] = scan->angle_min + (i*scan->angle_increment);

		filterLaser(laser_data, angle_data, scan->range_max, scan->range_min, 0.8, -0.8);

		float average_range = 0.0;

		BOOST_FOREACH(float range, laser_data) {
			average_range += range;
		}
		
		average_range /= laser_data.size();

		//ROS_INFO("AVERAGE: %lf, laser data size: %ld", average_range, laser_data.size());

		liftDoorOpenPrev = liftDoorOpen;
		if(average_range < 1.5) {
			liftDoorOpen = false;
		}
		else {
		  // ROS_INFO("LASER SAW DOOR OPEN");
		  liftDoorOpen = true;

		}
		
	}

	void CoroLiftExekutor::actionThread() {
		
		ROS_INFO("CoroLiftExekutor has started.\n");

		std::vector <std::string> params = extractParamStrings(getParams().c_str());
		if(params[0].find("ENTER") != std::string::npos || params[0].find("enter") != std::string::npos) {
			this->laser_sub_ = nh_.subscribe ("base_scan", 1, &CoroLiftExekutor::laserCallback, this);
		}
		else {
			this->laser_sub_ = nh_.subscribe ("base_scan_rear", 1, &CoroLiftExekutor::laserCallback, this);
		}

		while(ros::ok()) {
			ros::spinOnce();
			if(liftDoorOpen && !liftDoorOpenPrev) {
				ROS_INFO("Door state went from close to open.");
				liftDoorOpenPrev = liftDoorOpen;
				break;
			}
			else if(liftDoorOpen && (params[0].find("ENTER") != std::string::npos || params[0].find("enter") != std::string::npos)) {
			    ROS_INFO("Door is open. Entering lift.");
			    liftDoorOpenPrev = liftDoorOpen;
			    break;
			}
		}

		this->laser_sub_.shutdown();

		if(params[0].find("ENTER") != std::string::npos || params[0].find("enter") != std::string::npos)
			enterLift(params[1]);
		else
			exitLift(params[1]);
	}

	void CoroLiftExekutor::filterLaser (std::vector <float>& laser_data, std::vector <float>& angle_data, const float& max_range, const float& min_range, const float& max_angle, const float& min_angle)
	{
		std::vector<float>::iterator j = angle_data.begin();
		for(std::vector<float>::iterator i = laser_data.begin(); i != laser_data.end(); i++)
		{
			if(*i >= max_range || *i <= min_range || *j > max_angle || *j < min_angle)
			{
				laser_data.erase(i);
				i--;
				angle_data.erase(j);
				j--;
			}
			j++;
		}
	}

	void CoroLiftExekutor::exitLift(const std::string& floor_no) {
		ROS_INFO("EXIT LIFT");
		moveTo(std::string("outside_elevator_") + floor_no, std::string("backward"));
	}

	void CoroLiftExekutor::enterLift(const std::string& floor_no) {
		ROS_INFO("ENTER LIFT");
		moveTo(std::string("elevator_") + floor_no, std::string("forward"));
	}

	void CoroLiftExekutor::moveTo(const std::string& move_to_location, const std::string& direction) {

		std::string old_driving_dir;
		nh_.getParam("move_base/driving_direction", old_driving_dir);
		
		std::vector <double> object_location = cam_interface::getObjectReachablePositionFromCAM(move_to_location);
		move_base_msgs::MoveBaseGoal the_goal;
		the_goal.target_pose.header.frame_id = "map";
		the_goal.target_pose.header.stamp = ros::Time::now();
		the_goal.target_pose.pose.position.x = object_location[0];
		the_goal.target_pose.pose.position.y = object_location[1];
		the_goal.target_pose.pose.orientation.w = cos(object_location[2]/2);
		the_goal.target_pose.pose.orientation.z = sin(object_location[2]/2);

		nh_.setParam("move_base/driving_direction", direction);

		mb_client_.waitForServer(ros::Duration(30.0));

		if(!mb_client_.isServerConnected()) {
			ROS_INFO("Couldn't find the move_base action server in 30 seconds.");
			setState(FAILED);
			nh_.setParam("move_base/driving_direction", old_driving_dir);
			return;
		}

		ROS_INFO("Sending goal.");

		mb_client_.sendGoal(the_goal);

		mb_client_.waitForResult(ros::Duration(30.0));

		if(mb_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			ROS_INFO("Successfully executed the action on the robot.");
			setState(COMPLETED);
			sleep(1);
		}
		else
		{
			ROS_INFO("Attempted! But, something cocked-up and we failed!");
			setState(FAILED);
			sleep(1);
		}

		// SET OLD PARAMETERS BACK.

		nh_.setParam("move_base/driving_direction", old_driving_dir);
	}

}
