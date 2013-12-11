
#include <ros/ros.h>
#include <decision_making/SynchCout.h>
#include <decision_making/BT.h>
#include <decision_making/FSM.h>
#include <decision_making/ROSTask.h>
#include <decision_making/DecisionMaking.h>

using namespace std;
using namespace decision_making;

EventQueue* mainEventQueue;
struct MainEventQueue{
	MainEventQueue(){ mainEventQueue = new RosEventQueue();	}
	~MainEventQueue(){ delete mainEventQueue; }
};


bool leftBumper, rightBumper, wallSensor;
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

void run_fsm(){
	FsmRoomba(NULL, mainEventQueue);
}


void EVENTS_GENERATOR(){
	Event spec[]={"---","REPEAT"};
	int i=0;
	boost::this_thread::sleep(boost::posix_time::seconds(1));
	while(true and ros::ok()){
		Event t = spec[i];
		if(t == "REPEAT"){ i=1; t=spec[0]; }else i++;
		cout << endl << t<<" -> ";
		mainEventQueue->riseEvent(t);
		boost::this_thread::sleep(boost::posix_time::seconds(1));
	}
	mainEventQueue->close();
}

int main(int argc, char** argv){

	ros::init(argc, argv, "Roomba");
	MainEventQueue meq;
	ros_decision_making_init(argc, argv);

	boost::thread_group threads;
	threads.add_thread(new boost::thread(boost::bind(&run_fsm)));
	threads.add_thread(new boost::thread(boost::bind(&EVENTS_GENERATOR)));

	ros::spin();
	threads.join_all();

	return 0;
}


