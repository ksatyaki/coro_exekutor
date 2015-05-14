/*
 * coro_roller_exekutor.cpp
 *
 *  Created on: Jan 26, 2015
 *      Author: ace
 */

#include "exekutor/coro_roller_exekutor.h"

namespace exekutor
{

CoroRollerExekutor::CoroRollerExekutor(std::string robot_name, std::string action_name) :
		ActionExekutor (robot_name, action_name)
{
	roller_client_ = nh_.serviceClient <cognidrive_ros::Roller> ("Roller");
	roller_status_sub_ = nh_.subscribe("roller_status", 5, &CoroRollerExekutor::rollerStatusCallback, this);

	roller_status_prev_.reset();
}

void CoroRollerExekutor::rollerStatusCallback(const std_msgs::String::ConstPtr& rollerStatus)
{
	roller_status_prev_ = roller_status_;
	roller_status_ = rollerStatus;
}

CoroRollerExekutor::~CoroRollerExekutor()
{
}

void CoroRollerExekutor::actionThread()
{
	ROS_INFO("RollerExekutor has started.");
	std::string rollerDirection = getParamTuple().data;

	cognidrive_ros::Roller message;

	for(int i = 0; i<rollerDirection.size(); i++)
		rollerDirection[i] = toupper(rollerDirection[i]);

	if(rollerDirection.compare("LOAD") == 0)
	{
		message.request.type = cognidrive_ros::Roller::Request::IN;
		ROS_INFO("Moving roller in (LOAD).");
	}
	else if (rollerDirection.compare("UNLOAD") == 0)
	{
		message.request.type = cognidrive_ros::Roller::Request::OUT;
		ROS_INFO("Moving roller out (UNLOAD).");
	}

	roller_client_.call(message);

	while(true)
	{
		ros::spinOnce();
		if(!roller_status_ || !roller_status_prev_)
			continue;
		else
		{
			if(message.request.type == cognidrive_ros::Roller::Request::OUT && roller_status_prev_->data.compare("MOVING_OUT") == 0 && roller_status_->data.compare("OFF") == 0)
				break;
			else if(message.request.type == cognidrive_ros::Roller::Request::IN && roller_status_prev_->data.compare("MOVING_IN") == 0 && roller_status_->data.compare("OFF") == 0)
				break;
		}

	}

	setState(COMPLETED);
	ROS_INFO("Successfully executed the action.");
}

} /* namespace exekutor */


