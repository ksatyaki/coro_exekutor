/**
 * @file   coro_lift_exekutor.h
 * @author Chittaranjan S Srinivas
 * 
 * @brief  See the source file.
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

#ifndef CORO_LIFT_EXEKUTOR_H_
#define CORO_LIFT_EXEKUTOR_H_

#include <exekutor/action_exekutor.h>
#include <ros/ros.h>
#include <string>
#include <sensor_msgs/LaserScan.h>
#include <boost/foreach.hpp>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

namespace exekutor {

	/**
	 * Implements the "enter lift" and "exit lift behaviours".
	 * This class derives from ActionExekutor.
	 */
	class CoroLiftExekutor : public ActionExekutor {

	protected:

		ros::Subscriber laser_sub_;

		bool liftDoorOpen;
		bool liftDoorOpenPrev;
		
		/**
		 * The implementation of actionThread() function.
		 * This is called from ActionExekutor's startActionThread().
		 * This is where we create the client to the move_base action.
		 */
		virtual void actionThread();

		actionlib::SimpleActionClient <move_base_msgs::MoveBaseAction> mb_client_;

		/**
		 * Execute the enter lift behaviour. 
		 */
		void enterLift(const std::string& floor_no);

		/**
		 * Execute the exit lift behaviour.
		 */
		void exitLift(const std::string& floor_no);

		/** 
		 * Despatch a moveto to exekutor.
		 * @param move_to_location The location to moveto. 
		 */
		void moveTo(const std::string& move_to_location, const std::string& direction);

		/** 
		 * Filter out the data based on range and heading. 
		 * @param laser_data The vector of range values.
		 * @param angle_data The vector of heading values.
		 * @param max_range The range value above which all values are ignored.
		 * @param min_range The range value below which all values are ignored.
 		 * @param min_angle The angle value above which all values are ignored.
		 * @param min_angle The angle value below which all values are ignored.		 
		 */
		void filterLaser (std::vector <float>& laser_data, std::vector <float>& angle_data, const float& max_range, const float& min_range, const float& max_angle, const float& min_angle);

		void laserCallback(const sensor_msgs::LaserScanConstPtr& scan);

	public:
		/**
		 * Constructor. Takes robot_name and action_name as parameters. 
		 */
		CoroLiftExekutor(std::string robot_name = "coro",
						 std::string action_name = "lift");
		
		
	};
}

#endif
