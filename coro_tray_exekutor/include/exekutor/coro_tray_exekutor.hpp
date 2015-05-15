#ifndef CORO_TRAY_EXEKUTOR_HPP
#define CORO_TRAY_EXEKUTOR_HPP

#include <exekutor/action_exekutor.h>
#include <string>

namespace exekutor {
	class CoroTrayExekutor : public ActionExekutor {
	public:
		CoroTrayExekutor(std::string robot_name = "coro", std::string action_name = "tray");
	protected:
		std::string status_value_;
		static void statusCallback(PeisTuple* tuple, void* _this_);
		void actionThread();
	};
} 
#endif
