#include <exekutor/coro_tray_exekutor.hpp>

namespace exekutor {
	CoroTrayExekutor::CoroTrayExekutor(std::string robot_name, std::string action_name):
		ActionExekutor (robot_name, action_name)
	{
		peiskmt_subscribe(1797, "tray_occupied");
	}

	void CoroTrayExekutor::statusCallback(PeisTuple* tuple, void* _this_) {
		CoroTrayExekutor* this_ptr = static_cast <CoroTrayExekutor*> (_this_);
		this_ptr->status_value_ = tuple->data;
	}

	void CoroTrayExekutor::actionThread()
	{
		ROS_INFO("CoroTrayExekutor has started!");
		status_value_ = "";
		std::string option = this->getParamTuple().data;
		int counter = 200;

		PeisCallbackHandle callback = peiskmt_registerTupleCallback(1797, "tray_occupied", static_cast<void*>(this), &CoroTrayExekutor::statusCallback);
		
		if(option.compare("LOAD") == 0 || option.compare("load") == 0) {
			ROS_INFO("Waiting for tray to be loaded.");
			while(status_value_.compare("true") != 0 && counter == 0) {
				usleep(100000);
				counter--;
			}
		}
		else {
			ROS_INFO("Waiting for tray to be unloaded.");
			while(status_value_.compare("false") != 0 && counter == 0) {
				usleep(100000);
				counter--;
			}
		}

		peiskmt_unregisterTupleCallback(callback);

		if(counter == 0) {
			setResult("TIMED OUT");
			setState(FAILED);
			ROS_INFO("FAILURE!");
			return;
		}
		else {
			setResult("SUCCESS");
			setState(COMPLETED);
			ROS_INFO("SUCCESS!");
			return;
		}
	}
}
