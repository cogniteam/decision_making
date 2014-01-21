
#include <iostream>

#include <ros/ros.h>

#include <decision_making/SynchCout.h>
#include <decision_making/BT.h>
#include <decision_making/FSM.h>
#include <decision_making/ROSTask.h>
#include <decision_making/DecisionMaking.h>

using namespace std;
using namespace decision_making;

bool MissionLoaded;
FSM(MissionActive)
{
	FSM_STATES
	{
		MissionSpooling,
		MissionPaused,
		MissionAborted,
		MissionFinished,
	}
	FSM_START(MissionSpooling);
	FSM_BGN
	{
		FSM_STATE(MissionSpooling)
		{
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/CompleteMission", FSM_NEXT(MissionFinished));
				FSM_ON_EVENT("/AbortMission", FSM_NEXT(MissionAborted));
				FSM_ON_EVENT("/PauseMission", FSM_NEXT(MissionPaused));
			}
		}
		FSM_STATE(MissionPaused)
		{
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/CompleteMission", FSM_NEXT(MissionFinished));
				FSM_ON_EVENT("/AbortMission", FSM_NEXT(MissionAborted));
				FSM_ON_EVENT("/ResumeMission", FSM_NEXT(MissionSpooling));
			}
		}
		FSM_STATE(MissionAborted)
		{
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/StartMission", FSM_NEXT(MissionSpooling));
			}
		}
		FSM_STATE(MissionFinished)
		{
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/StartMission", FSM_NEXT(MissionSpooling));
			}
		}
	}
	FSM_END
}

FSM(Mission_ON)
{
	FSM_STATES
	{
		NoMissionLoaded,
		MissionPending,
		MissionActive
	}
	FSM_START(NoMissionLoaded);
	FSM_BGN
	{
		FSM_STATE(NoMissionLoaded)
		{
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/MissionLoaded", FSM_NEXT(MissionPending));
			}
		}
		FSM_STATE(MissionPending)
		{
			FSM_TRANSITIONS
			{
				//NOTE: It's not clear for transition from MissionPending to NoMissionLoaded
				FSM_ON_EVENT("/DeleteMission", FSM_NEXT(NoMissionLoaded));
				FSM_ON_EVENT("/ClearMissionBuffer", FSM_NEXT(NoMissionLoaded));
				FSM_ON_EVENT("/not_MissionLoaded", FSM_NEXT(NoMissionLoaded));

				FSM_ON_EVENT("/StartMission", FSM_NEXT(MissionActive));
			}
		}
		FSM_STATE(MissionActive)
		{
			FSM_CALL_FSM(MissionActive)
			FSM_TRANSITIONS
			{
				//NOTE: It's not clear for transition from MissionActive to NoMissionLoaded
				FSM_ON_EVENT("/DeleteMission", FSM_NEXT(NoMissionLoaded));
				FSM_ON_EVENT("/Stendby", FSM_NEXT(NoMissionLoaded));
				FSM_ON_EVENT("/ClearMissionBuffer", FSM_NEXT(NoMissionLoaded));
				FSM_ON_EVENT("not_MissionLoaded", FSM_NEXT(NoMissionLoaded));
			}
		}

	}
	FSM_END
}

FSM(Mission)
{
	FSM_STATES
	{
		OFF,
		ON
	}
	FSM_START(OFF);
	FSM_BGN
	{
		FSM_STATE(OFF)
		{
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/SystemActivation", FSM_NEXT(ON));
			}
		}
		FSM_STATE(ON)
		{
			FSM_CALL_FSM(Mission_ON)
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/PowerOff", FSM_NEXT(OFF));
			}
		}

	}
	FSM_END
}

int main(int argc, char** argv){

	ros::init(argc, argv, "fsm_mission");
	ros_decision_making_init(argc, argv);

	ros::AsyncSpinner spinner(2);
	spinner.start();

	ROS_INFO("Starting mission...");
	FsmMission(NULL, new RosEventQueue());

	spinner.stop();

	return 0;
}


