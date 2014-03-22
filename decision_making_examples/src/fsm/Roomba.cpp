#include <iostream>

#include <decision_making/SynchCout.h>
#include <decision_making/BT.h>
#include <decision_making/FSM.h>
#include <decision_making/ROSTask.h>
#include <decision_making/DecisionMaking.h>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Range.h>

using namespace std;
using namespace decision_making;

volatile bool leftBumper, rightBumper, wallSensor;

ros::Subscriber leftBumperSub;
ros::Subscriber rightBumperSub;
ros::Subscriber wallSensorSub;

EventQueue* mainEventQueue;
struct MainEventQueue{
	MainEventQueue(){ mainEventQueue = new RosEventQueue();	}
	~MainEventQueue(){ delete mainEventQueue; }
};

FSM(Roomba)
{
	enum STATES
	{
		findWall,
		turnSx,
		correctSx,
		correctDx
	}
	FSM_START(findWall);
	FSM_BGN
	{
		FSM_STATE(findWall)
		{
			FSM_TRANSITIONS
			{
				FSM_ON_CONDITION( leftBumper==1||rightBumper==1 , FSM_NEXT(turnSx));
			}
		}
		FSM_STATE(turnSx)
		{
			FSM_TRANSITIONS
			{
				FSM_ON_CONDITION( leftBumper==0&&rightBumper==0 , FSM_NEXT(correctDx));
			}
		}
		FSM_STATE(correctSx)
		{
			FSM_TRANSITIONS
			{
				FSM_ON_CONDITION( leftBumper==1||rightBumper==1 , FSM_NEXT(turnSx));
				FSM_ON_CONDITION( wallSensor==0 , FSM_NEXT(correctDx));
			}
		}
		FSM_STATE(correctDx)
		{
			FSM_TRANSITIONS
			{
				FSM_ON_CONDITION( leftBumper==1||rightBumper==1 , FSM_NEXT(turnSx));
				FSM_ON_CONDITION( wallSensor==1 , FSM_NEXT(correctSx));
			}
		}
	}
	FSM_END
}

void onLeftBumperMessage(const std_msgs::Bool::Ptr& bumperState) {
    ROS_INFO("Left bumper state = %s", bumperState->data ? "Active" : "Not active");
    leftBumper = bumperState->data;
}

void onRightBumperMessage(const std_msgs::Bool::Ptr& bumperState) {
    ROS_INFO("Right bumper state = %s", bumperState->data ? "Active" : "Not active");
    rightBumper = bumperState->data;
}

void onWallSensorMessage(const sensor_msgs::Range::Ptr& sensor) {
     wallSensor = sensor->range < 0.3;
    ROS_INFO("Wall sensor range = %f, state = %s", sensor->range, wallSensor ? "Active" : "Not active");
}

int main(int argc, char** argv){

	ros::init(argc, argv, "fsm_roomba");
	MainEventQueue meq;
	ros_decision_making_init(argc, argv);

	ros::NodeHandle node;
	leftBumperSub = node.subscribe("/roomba/left_bumper", 1, onLeftBumperMessage);
	rightBumperSub = node.subscribe("/roomba/left_bumper", 1, onRightBumperMessage);
	wallSensorSub = node.subscribe("/roomba/wall_sensor", 1, onWallSensorMessage);

	ros::AsyncSpinner spinner(2);
	spinner.start();

	ROS_INFO("Starting roobma...");

	/**
	 * Blocking call
	 */
	mainEventQueue->async_spin();
	FsmRoomba(NULL, mainEventQueue);
	mainEventQueue->close();
	
	spinner.stop();
	ROS_INFO("Roomba done");

	return 0;
}


