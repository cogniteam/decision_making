
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

#include "BT.h"
#include "FSM.h"


//==================== CONNECTION TO REAL TASKS ==============================


TaskResult callTask(std::string task_address, const CallContext& call_ctx, EventQueue& queue){
	DMDEBUG( cout<<" TASK("<<task_address<<":CALL) " ;)
	while(true)
	{
		Event e = queue.waitEvent();
		if(not e){
			DMDEBUG( cout<<" TASK("<<task_address<<":TERMINATED) " ; )
			return TaskResult::TERMINATED();
		}
		if( e=="/SUCCESS" or e=="/FAIL" or e=="/GO" ){
			Event new_event ( Event(""+e.event_name(), call_ctx) );
			DMDEBUG( cout<<" TASK("<<task_address<<":"<<new_event<<") "; )
			queue.riseEvent(new_event);
		}

		if(e=="/GO" and task_address=="T3") return TaskResult::SUCCESS();
		//return TaskResult::FAIL();
	}
	return TaskResult::FAIL();
}

#define CALL_REMOTE(NAME, CALLS, EVENTS) boost::bind(&callTask, #NAME, CALLS, EVENTS)


#include "DecisionMaking.h"

//=============================================================================


FSM(TEST1){
	enum STATt{
		C,
		D,
		S
	}
	FSM_START(C);
	FSM_BGN{
		FSM_STATE( C ){
			FSM_CALL_TASK(C);
			FSM_TRANSITIONS{
				FSM_ON_EVENT(C/SUCCESS, FSM_NEXT(D));
				FSM_ON_EVENT(C/FAIL, FSM_NEXT(D));
			}
		}
		FSM_STATE( D ){
			FSM_CALL_TASK(D);
			FSM_TRANSITIONS{
//				FSM_ON_EVENT(D/SUCCESS, FSM_NEXT(D));
//				FSM_ON_EVENT(D/FAIL, FSM_NEXT(D));
			}
		}
		FSM_STATE( S ){
			FSM_STOP(SUCCESS, TaskResult::SUCCESS());
			FSM_TRANSITIONS{}
		}
	}
	FSM_END
}


BT_BGN(BT1){
	BT_PAR_BGN(P1){
		BT_CALL_TASK(T1);
		BT_CALL_TASK(T2);
	}
	BT_PAR_END(P1);
}
BT_END(BT1);


//struct _bt_function_struct{
//TaskResult _bt_function(std::string task_address, CallContext& call_ctx, EventQueue& queue){
//	BTContext _tmp_context;
//	BT_ROOT_BGN(bt_from_fsm, __tmp_event_queue()){
//		call_ctx.pop();
//		BT_PAR_BGN(TMP){
//			call_ctx.pop();
//			BT_TASK_BGN(STOP_CONDITION){
//				while(not isTerminated())
//				{
//					Event e = events.waitEvent();
//					if(not e){
//						cout<<" (STOP_CONDITION:TERMINATED) ";
//						BT_TASK_RESULT( TaskResult::TERMINATED() );
//						break;
//					}
//				}
//			}
//			BT_TASK_END(STOP_CONDITION);
//			BT_CALL_BT(BT1);
//		}
//		BT_PAR_END(TMP);
//	}
//	BT_END(bt_from_fsm) bt_from_fsm(_tmp_context, call_ctx, queue);
//	TaskResult res = bt_from_fsm.run();
//	cout<<"(fsm from function finished)";
//	return res;
//}
//} _bt_function_struct_instance;
//#define CALL_BT_FUNCTION(NAME, CALLS, EVENTS) boost::bind(&_bt_function_struct::_bt_function, &_bt_function_struct_instance, #NAME, CALLS, EVENTS)




//X(
FSM(TEST){
	enum STATt{
		A,
		B,
		STOP
	}
	FSM_START(A);
	FSM_BGN{
		FSM_STATE( A ){
			FSM_CALL_TASK(TA);
			//FSM_CALL_TASK(TB);
			FSM_CALL_BT(BT1);


			//FSM_RISE(GO_TO_STOP)
			FSM_TRANSITIONS{
				FSM_PRINT_EVENT;
				FSM_ON_EVENT(TA/GO, FSM_NEXT(STOP));
				//FSM_ON_EVENT(/A/SUCCESS, FSM_NEXT(B));
				//FSM_ON_EVENT(/A/FAIL, FSM_NEXT(B));//
				//FSM_ON_EVENT(GO_TO_STOP, FSM_NEXT(STOP));
			}
		}
		FSM_STATE( B ){
			FSM_CALL_FSM(TEST1);
			FSM_TRANSITIONS{
				FSM_PRINT_EVENT;
				FSM_ON_EVENT(TEST1/D/SUCCESS, FSM_NEXT(A));
				//FSM_ON_EVENT(FAIL, FSM_NEXT(B));
			}
		}
		FSM_STATE( STOP ){

			FSM_TRANSITIONS
		}
	}
	FSM_END
}

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



