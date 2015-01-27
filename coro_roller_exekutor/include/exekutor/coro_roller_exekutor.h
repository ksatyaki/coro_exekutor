/*
 * coro_roller_exekutor.h
 *
 *  Created on: Jan 26, 2015
 *      Author: ace
 */

#ifndef CORO_ROLLER_EXEKUTOR_H_
#define CORO_ROLLER_EXEKUTOR_H_

#include <exekutor/action_exekutor.h>
#include <algorithm>
#include <cognidrive_ros/Roller.h>
#include <std_msgs/String.h>

namespace exekutor
{

class CoroRollerExekutor: public ActionExekutor
{
	/**
	 * A cakkback for the roller status.
	 */
	void rollerStatusCallback(const std_msgs::String::ConstPtr& rollerStatus);

	/**
	 * A Subscriber to the message.
	 */
	ros::Subscriber roller_status_sub_;

	/**
	 * A client to the Roller in and out services.
	 */
	ros::ServiceClient roller_client_;


	/**
	 * The action thread function - overloaded.
	 */
	void actionThread();

	/**
	 * The globally maintained roller status.
	 */
	std_msgs::String::ConstPtr roller_status_;

	/**
	 * The previous status.
	 */
	std_msgs::String::ConstPtr roller_status_prev_;

public:
	/**
	 * Constructor. Calls the "super" constructor.
	 */
	CoroRollerExekutor(std::string robot_name, std::string action_name);

	virtual ~CoroRollerExekutor();
};

} /* namespace exekutor */

#endif /* CORO_ROLLER_EXEKUTOR_H_ */
