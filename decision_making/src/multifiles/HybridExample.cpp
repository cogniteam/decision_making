
#include <iostream>


#include <boost/thread.hpp>
#include <deque>
#include <vector>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>
#include <boost/shared_ptr.hpp>


using namespace std;

#include <SynchCout.h>
#include <EventSystem.h>

using namespace decision_making;


EventQueue mainEventQueue;


void EVENTS_GENERATOR(){
//	Event spec[]={"SUCCESS", "FAIL", "SUCCESS", "FAIL", "SUCCESS", "FAIL", "SUCCESS", "FAIL", "GO", "NOTHING"};
//	Event spec[]={"FAIL", "NOTHING"};
	Event spec[]={"SUCCESS","SUCCESS","SUCCESS","SUCCESS","SUCCESS","SUCCESS","GO", "NOTHING"};
	int i=0;
	boost::this_thread::sleep(boost::posix_time::seconds(1));
	while(true){
		Event t = spec[i];
		if(t == "NOTHING"){ i=1; t=spec[0]; }else i++;
		cout << endl << t<<" -> ";
		mainEventQueue.riseEvent(t);
		boost::this_thread::sleep(boost::posix_time::seconds(2));
	}
}


//==================== END OF SYMULATION OF REAL SYSTEM ======================

#define X(x) x
#define PAUSE boost::this_thread::sleep(boost::posix_time::seconds(1.3));
#define TIMES(N) for(size_t times=0;times<N;times++)



//==================== CONNECTION TO REAL TASKS ==============================


#include "custom_decision_making.h"


//=============================================================================


#include "BT_BT1.h"
#include "FSM_TEST.h"

void FSM_TEST(){
	FsmTEST(NULL,&mainEventQueue);
}

TaskResult BT_run_(){
	BT_ROOT_BGN(root, mainEventQueue){
		BT_PAR_BGN(P1){
//			BT_TASK_BGN(MY1){
//				cout<<"HELLO"<<endl;
//				FSM_TEST();
//				BT_TASK_RESULT( TaskResult::SUCCESS() );
//			}
//			BT_TASK_END(MY1);
			BT_CALL_FSM(TEST);

//			BT_SEQ_BGN(S1){
//				BT_TASK_ROS(T1);
//				BT_TASK_ROS(T2);
//			}
//			BT_SEQ_END(S1);
			BT_CALL_TASK(T3);
			BT_CALL_TASK(T4);
			BT_CALL_BT(BT1);
		}
		BT_PAR_END(P1);
	}
	BT_END(root) main;
	return main.run();
}


int main() {

boost::thread_group threads;
threads.add_thread(new boost::thread(boost::bind(&BT_run_)));
//threads.add_thread(new boost::thread(boost::bind(&FSM_TEST)));
threads.add_thread(new boost::thread(boost::bind(&EVENTS_GENERATOR)));

threads.join_all();

return 0;
}



