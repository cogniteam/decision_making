
#include <iostream>

#include <ros/ros.h>

#include <decision_making/SynchCout.h>
#include <decision_making/BT.h>
#include <decision_making/FSM.h>
#include <decision_making/ROSTask.h>
#include <decision_making/DecisionMaking.h>

using namespace std;
using namespace decision_making;

FSM(Turnstile)
{
	FSM_STATES
	{
		Locked,
		Unlocked
	}
	FSM_START(Locked);
	FSM_BGN
	{
		FSM_STATE(Locked)
		{
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/COIN", FSM_NEXT(Unlocked));
				FSM_ON_EVENT("/PUSH", FSM_NEXT(Locked));
			}
		}
		FSM_STATE(Unlocked)
		{
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/COIN", FSM_NEXT(Unlocked));
				FSM_ON_EVENT("/PUSH", FSM_NEXT(Locked));
			}
		}
	}
	FSM_END
}

int main(int argc, char** argv){

	ros::init(argc, argv, "fsm_turnstile");
	ros_decision_making_init(argc, argv);

	ros::AsyncSpinner spinner(2);
	spinner.start();

	ROS_INFO("Starting turnstile...");
	FsmTurnstile(NULL, new RosEventQueue());

	spinner.stop();

	return 0;
}


