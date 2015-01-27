#include <exekutor/coro_roller_exekutor.h>
#include <signal.h>

void handler(int signal_number)
{
	if(signal_number == SIGINT)
		printf("Ctrl-C caught...\n");
	else
		printf("Ctrl-Z caught...\n");

	ros::shutdown();
	peiskmt_shutdown();

	abort();
}

int main(int argn, char* args[])
{
	ros::init(argn, args, "coro_roller_exekutor");
	peiskmt_initialize (&argn, args);

	struct sigaction sa;
	memset(&sa, 0, sizeof(sa));
	sa.sa_handler = &handler;
	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGTSTP, &sa, NULL);

	exekutor::CoroRollerExekutor roller_exek ("coro", "roller");
	roller_exek.waitForLink();

	return 0;
}
