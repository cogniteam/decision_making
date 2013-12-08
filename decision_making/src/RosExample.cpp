
//#include <iostream>
//
//
//#include <boost/thread.hpp>
//#include <deque>
//#include <vector>
//#include <boost/thread/mutex.hpp>
//#include <boost/thread/condition.hpp>
//#include <boost/shared_ptr.hpp>


using namespace std;

#include "SynchCout.h"

#include "BT.h"
#include "FSM.h"
#include "ROSTask.h"
#include "DecisionMaking.h"

using namespace decision_making;

EventQueue mainEventQueue;

//#define X(x) x
//#define PAUSE boost::this_thread::sleep(boost::posix_time::seconds(1.3));
//#define TIMES(N) for(size_t times=0;times<N;times++)



FSM(RosTest)
{
	enum STATES{
		run_task,
		stop
	}
	FSM_START(run_task);
	FSM_BGN
	{
		FSM_STATE(run_task)
		{
			CONSTRAINTS(rt,
				$ interval 1;
				$ fail warn;
				core1 = {/scan/ranges[0:1]};
				core2 = {/scan/ranges[1:2]};
				core3 = {/scan/ranges[2:3]};
				core4 = {/scan/ranges[3:4]};
				cpu_th = {/scan/ranges[4:5]};
				core_th = {/scan/ranges[5:6]};
				average = SomeFunction(core1, core2, core3, core4);
				average < cpu_th;
				core1 < core_th;
				core2 < core_th;
				core3 < core_th;
				core4 < core_th;
			);
			FSM_CALL_TASK(MYTASK);
			FSM_TRANSITIONS
			{
				FSM_PRINT_EVENT;
				FSM_ON_EVENT(/GO, FSM_NEXT(stop));
			}
		}
		FSM_STATE(stop)
		{
			FSM_TRANSITIONS
			{
				FSM_PRINT_EVENT;
				FSM_ON_EVENT(/GO, FSM_NEXT(run_task));
			}
		}
	}
	FSM_END
}

void run_fsm(){
	FsmRosTest(0, &mainEventQueue);
}



void run_bt(){
	BT_ROOT_BGN(RosTest, mainEventQueue)
	{
		BT_CALL_TASK(MYTASK);
	}
	BT_END(RosTest) bt;
	bt.run();

	cout<<"[exit from thread]";
}




void EVENTS_GENERATOR(){
	Event spec[]={"SUCCESS","SUCCESS","SUCCESS","GO","SUCCESS","SUCCESS","SUCCESS","GO", "NOTHING"};
	int i=0;
	boost::this_thread::sleep(boost::posix_time::seconds(1));
	while(true and ros::ok()){
		Event t = spec[i];
		if(t == "NOTHING"){ i=1; t=spec[0]; }else i++;
		cout << endl << t<<" -> ";
		mainEventQueue.riseEvent(t);
		boost::this_thread::sleep(boost::posix_time::seconds(1));
	}
	mainEventQueue.close();
}

TaskResult tst_mytask(std::string task_address, const FSMCallContext& call_ctx, EventQueue& queue){
	cout<<" this this my task ";
	queue.riseEvent(Event("success", call_ctx));
	return TaskResult::SUCCESS();
}

#include <boost/filesystem.hpp>

int main(int a, char** aa){

	ros::init(a, aa, "RosExample");
	ros_decision_making_init(a, aa);

	cout<<"Path: "<<boost::filesystem::path(aa[0])<<endl;
	cout<<"Name: "<<ros::this_node::getName()<<endl;
	cout<<"Namespace: "<<ros::this_node::getNamespace()<<endl;


	boost::thread_group threads;

	MapResultEvent::map("MYTASK", 0, "success");
	//LocalTasks::registrate("MYTASK", tst_mytask);

	threads.add_thread(new boost::thread(boost::bind(&run_fsm)));
	threads.add_thread(new boost::thread(boost::bind(&EVENTS_GENERATOR)));

	threads.join_all();


	return 0;
}


