
#include <iostream>


#include <boost/thread.hpp>
#include <deque>
#include <vector>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>
#include <boost/shared_ptr.hpp>


using namespace std;

#include "SynchCout.h"
#include "EventSystem.h"

using namespace decision_making;


EventQueue mainEventQueue;

//=================== SYMULATION OF REAL SYSTEM ============================

//enum EVENT_t{
//	E_NOTHING, E_SUCCESS, E_FAIL
//};
//std::ostream& operator<<(std::ostream& o, EVENT_t t){
//	switch(t){
//	case E_SUCCESS: return o<<"SUCC";
//	case E_FAIL: return o<<"FAIL";
//	default: break;
//	}
//	return o<<"*";
//}
//
//boost::mutex events_mutex;
//boost::condition_variable on_new_event;
//std::deque<EVENT_t> events;
//bool events_system_stop = false;
//void addEvent(EVENT_t e){
//	boost::mutex::scoped_lock l(events_mutex);
//	events.push_back(e);
//	on_new_event.notify_one();
//}
//EVENT_t waitEvent(){
//	boost::mutex::scoped_lock l(events_mutex);
//	while(events_system_stop==false && events.empty())	on_new_event.wait(l);
//	if(events_system_stop) return E_NOTHING;
//	EVENT_t e = events.front();
//	events.pop_front();
//	return e;
//}

void EVENTS_GENERATOR(){
//	Event spec[]={"SUCCESS", "FAIL", "SUCCESS", "FAIL", "SUCCESS", "FAIL", "SUCCESS", "FAIL", "GO", "NOTHING"};
	Event spec[]={"SUCCESS", "NOTHING"};
	int i=0;
	boost::this_thread::sleep(boost::posix_time::seconds(1));
	while(true){
		Event t = spec[i];
		if(t == "NOTHING"){ i=1; t=spec[0]; }else i++;
		cout << endl << t<<" -> ";
		mainEventQueue.riseEvent(t);
		boost::this_thread::sleep(boost::posix_time::seconds(1));
	}
}
//void EVENTS_GENERATOR(){
//	//EVENT_t spec[]={E_SUCCESS, E_FAIL, E_SUCCESS, E_FAIL, E_SUCCESS, E_FAIL, E_SUCCESS, E_FAIL, E_NOTHING};
//	//EVENT_t spec[]={E_SUCCESS, E_FAIL, E_FAIL,E_SUCCESS,E_SUCCESS,E_SUCCESS,E_SUCCESS,E_SUCCESS,E_SUCCESS,E_SUCCESS,E_NOTHING};
//	//EVENT_t spec[]={E_SUCCESS, E_NOTHING};
//	EVENT_t spec[]={E_FAIL, E_NOTHING};
//	int i=0;
//	boost::this_thread::sleep(boost::posix_time::seconds(1));
//	while(true){
//		EVENT_t t = spec[i];
//		if(t == E_NOTHING){ i=1; t=spec[0]; }else i++;
//		std::cout<<endl<<t<<" -> ";
//		addEvent(t);
//		boost::this_thread::sleep(boost::posix_time::seconds(1));
//	}
//}

//==================== END OF SYMULATION OF REAL SYSTEM ======================

#define X(x) x
#define PAUSE boost::this_thread::sleep(boost::posix_time::seconds(1.3));
#define TIMES(N) for(size_t times=0;times<N;times++)

#include "BT.h"
//#include "ROSTask.h"

//==================== CONNECTION TO REAL TASKS ==============================

//FROM BT
TaskResult test_callTask(std::string task_address, const CallContext& call_ctx, EventQueue& queue){
	cout<<" TASK("<<task_address<<":CALL) ";
	Event e = mainEventQueue.waitEvent();
	if(e=="SUCCESS") return TaskResult::SUCCESS();
	return TaskResult::FAIL();
}


//FROM FSM
//void callTask(std::string task_address, const CallContext& call_ctx, EventQueue* queue){
//	cout<<" TASK("<<task_address<<":CALL) ";
//	while(true)
//	{
//		Event e = queue->waitEvent();
//		if(not e){
//			cout<<" TASK("<<task_address<<":TERMINATED) ";
//			return;
//		}
//		if( e=="/SUCCESS" or e=="/FAIL" or e=="/GO" ){
//			Event new_event ( Event(""+e.event_name(), call_ctx) );
//			cout<<" TASK("<<task_address<<":"<<new_event<<") ";
//			queue->riseEvent(new_event);
//		}
//	}
//}

//#define CALL_REMOTE(NAME) test_callTask(#NAME, call_ctx, events)
#define CALL_REMOTE(NAME, CALLS, EVENTS) boost::bind(&test_callTask, #NAME, CALLS, EVENTS)

//=============================================================================


//BT_BGN(ST1_){
//	PAR_BGN(P1){
//		TASK_ROS(T1);
//		TASK_ROS(T2);
//	}
//	PAR_END(P1);
//}
//BT_END(ST1_);
//
//
//
//TaskResult BT_run_version_with_subtree_(){
//	ROOT_BT_BGN(root){
//		SEQ_BGN(S1){
//			CALL_BT(ST1_);
//			TASK_ROS(T3);
//			TASK_ROS(T4);
//		}
//		SEQ_END(S1);
//	}
//	BT_END(root) main;
//	return main.run();
//}

TaskResult BT_run_(){
	BT_ROOT_BGN(root, mainEventQueue){
		BT_PAR_BGN(P1){
			BT_TASK_BGN(MY1){
				cout<<"HELLO"<<endl;
				//createRosTaskCaller(selfPtrForRosTaskCaller);
				BT_TASK_RESULT( TaskResult::SUCCESS() );
			}
			BT_TASK_END(MY1);
			BT_SEQ_BGN(S1){
				BT_CALL_TASK(T1);
				BT_CALL_TASK(T2);
			}
			BT_SEQ_END(S1);
			BT_CALL_TASK(T3);
			BT_CALL_TASK(T4);
		}
		BT_PAR_END(P1);
	}
	BT_END(root) main;
	return main.run();
}


////TREE(ST1,
////	PAR(P1,
////		CALL_TASK(T1);
////		CALL_TASK(T2);
////	);
////);
////
////EVENT_t BT_run_version_with_subtree(){
////	ROOT(root,
////		SEQ(S1,
////			USE_TREE(ST1);
////			CALL_TASK(T3);
////			CALL_TASK(T4);
////		);
////	) main;
////	return main.run();
////}
////
////
////EVENT_t BT_run_v1(){
////	ROOT(root,
////		PAR(P1,
////			TASK(MY1,
////				std::cout<<"HELLO"<<endl;
////				ret_value = E_SUCCESS;
////			);
////			SEQ(S1,
////				CALL_TASK(T1);
////				CALL_TASK(T2);
////			);
////			CALL_TASK(T3);
////			CALL_TASK(T4);
////		);
////	) main;
////	return main.run();
////}
////
////EVENT_t BT_run(){
////
////	NEW_CONTEXT(
////		int a;
////		int a1;
////		double a2;
////		std::string a3;
////	);
////	ROOT(root,
////		SEQ(S1,
////
////			TASK(t1,
////				context.a+=1;
////			);
////			NOT(
////				TASK(MY1,
////					TASK(t1,
////						context.a+=1;
////						cout<<"a="<<context.a<<endl;
////					);
////					LAST_NODE->run();
////					for(int i=0;i<3;i++){
////						CALL_TASK(T1);
////						EVENT_t e=BT_NODE(T1)->run();
////						TASK_RESULT(e);
////					}
////				);
////			);
////			RENAME_CONTEXT(SuperContext);
////			NEW_CONTEXT(
////				SuperContextType& aa;
////				int b;
////				BTContext(SuperContextType& a):aa(a),b(10){}
////			)(SuperContext);
////			TASK(t2,
////				cout<<"aa="<<context.aa.a<<":"<<context.b<<endl;
////			);
////		);
////	) main;
////	return main.run();
////}
//

int main() {

boost::thread_group threads;
threads.add_thread(new boost::thread(boost::bind(&BT_run_)));
//threads.add_thread(new boost::thread(boost::bind(&BT_run_version_with_subtree_)));
threads.add_thread(new boost::thread(boost::bind(&EVENTS_GENERATOR)));

threads.join_all();

return 0;
}



