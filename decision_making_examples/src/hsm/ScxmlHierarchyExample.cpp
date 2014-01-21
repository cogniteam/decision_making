
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

FSM(Superstate)
{
	FSM_STATES
	{
		Start, Process
	}
	FSM_START(Start);
	FSM_BGN
	{
		FSM_STATE(Start)
		{
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/begin", FSM_NEXT(Process));
				FSM_ON_EVENT("/end", FSM_RISE("Start/end"));
				FSM_ON_EVENT("/data", FSM_RISE("Start/data"));
			}
		}
		FSM_STATE(Process)
		{
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/data", FSM_NEXT(Process));
				FSM_ON_EVENT("/end", FSM_RISE("Process/end"));
				FSM_ON_EVENT("/data", FSM_RISE("Process/data"));
			}
		}
	}
	FSM_END
}

FSM(ScxmlHierarchyExample)
{
	FSM_STATES
	{
		Success, Error, Superstate
	}
	FSM_START(Superstate);
	FSM_BGN
	{
		FSM_STATE(Success)
		{
			FSM_TRANSITIONS
		}
		FSM_STATE(Error)
		{
			FSM_TRANSITIONS
		}
		FSM_STATE(Superstate)
		{
			FSM_CALL_FSM(Superstate);
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/error", FSM_NEXT(Error));
				FSM_ON_EVENT("Superstate/error", FSM_NEXT(Error));
				FSM_ON_EVENT("Superstate/Start/end", FSM_NEXT(Error));
				FSM_ON_EVENT("Superstate/Start/data", FSM_NEXT(Error));
				FSM_ON_EVENT("Superstate/Process/begin", FSM_NEXT(Error));
				FSM_ON_EVENT("Superstate/Process/end", FSM_NEXT(Success));
			}
		}

	}
	FSM_END
}

void run_fsm(){
	FsmScxmlHierarchyExample(NULL, mainEventQueue);
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

	ros::init(argc, argv, "ScxmlHierarchyExample");
	MainEventQueue meq;
	ros_decision_making_init(argc, argv);

	boost::thread_group threads;
	threads.add_thread(new boost::thread(boost::bind(&run_fsm)));
	threads.add_thread(new boost::thread(boost::bind(&EVENTS_GENERATOR)));

	ros::spin();
	threads.join_all();

	return 0;
}


