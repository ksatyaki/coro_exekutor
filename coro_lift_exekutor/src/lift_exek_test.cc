/*
 * lift_exekutor_test.cc
 *
 *  Chittaranjan
 */

#include <exekutor/coro_lift_exekutor.hpp>

int main(int argn, char* args[])
{
	peiskmt_initialize(&argn, args);
	ros::init(argn, args, "test_coro_lift_exekutor");
	
	exekutor::CoroLiftExekutor l_e_;
	while(ros::ok()) { sleep(1); }

	return 0;
}
