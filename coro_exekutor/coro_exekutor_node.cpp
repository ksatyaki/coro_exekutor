#include <exekutor/move_to_exekutor.h>
#include <exekutor/move_to_simple_exekutor.h>
#include <cam_interface/cam_interface.h>
#include <exekutor/load_map_exekutor.h>
#include <exekutor/coro_roller_exekutor.h>
#include <exekutor/miradock_exekutor.h>


#include<signal.h>

void handler(int signal_number)
{
	if(signal_number == SIGINT)
	{
		printf("Ctrl-C caught...\n");
		ros::shutdown();
		peiskmt_shutdown();
		abort();

	}
	else
	{
		printf("Ctrl-Z caught...\n");
		ros::shutdown();
		peiskmt_shutdown();
		abort();
	}
}


int main(int argn, char* args[])
{
	peiskmt_initialize(&argn, args);
	ros::init(argn, args, "exekutor");

	cam_interface::subscribeToCAM();

	struct sigaction sa;
	memset(&sa, 0, sizeof(sa));
	sa.sa_handler = &handler;
	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGTSTP, &sa, NULL);

	exekutor::MoveToExekutor MTX("coro", "moveto");
	exekutor::MoveToSimpleExekutor MTSX("coro", "movetosimple");
	exekutor::MiradockExekutor MDX("coro", "miradock");
	exekutor::LoadMapExekutor LMX("coro", "loadmap");
	exekutor::CoroRollerExekutor CRX ("coro", "roller");

	exekutor::ActionExekutor::waitForLink();

	return 0;
}

