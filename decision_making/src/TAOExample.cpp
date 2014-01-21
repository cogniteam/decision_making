//============================================================================
// Name        : DMExample.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>


#include "SynchCout.h"
#include "EventSystem.h"
#include "TAO.h"
using namespace decision_making;

EventQueue mainEventQueue;

#define CALL_REMOTE(NAME, CALLS, EVENTS) boost::bind(&callTask, #NAME, CALLS, EVENTS)

#include "DecisionMaking.h"

void callTask(std::string task_address, const CallContext& call_ctx, EventQueue& queue){
	cout<<" TASK("<<task_address<<":CALL) ";
	while(true)
	{
		Event e = queue.waitEvent();
		if(not e){
			cout<<" TASK("<<task_address<<":TERMINATED) ";
			return;
		}
		if( e=="/SUCCESS" or e=="/FAIL" or e=="/GO" ){
			Event new_event ( Event(""+e.event_name(), call_ctx) );
			cout<<" TASK("<<task_address<<":"<<new_event<<") ";
			queue.riseEvent(new_event);
			if(e=="/FAIL" and task_address=="RRR") return;
		}
	}
}



#define X(...) __VA_ARGS__


class NextSelectFirst:public decision_making::ProtocolNext{
public:
	NextSelectFirst(int& res, decision_making::CallContext* call_context, decision_making::EventQueue* events):ProtocolNext(res, call_context, events){}
	bool decide(){
		cout<<"[decide for next of "<<call_context->str()<<" ]";
		for(size_t i=0;i<options.size();i++){
			if(options[i].isReady){
				cout<<" set next state = "<<options[i].id <<": "<<options[i].name<<endl;
				result = options[i].id;
				return true;
			}
		}
		return false;
	}
};

class AllocFirstReady:public decision_making::ProtocolAllocation{
public:
	AllocFirstReady(int& res, decision_making::CallContext* call_context, decision_making::EventQueue* events):ProtocolAllocation(res, call_context, events){}
	bool decide(){
		cout<<"[decide for next of "<<call_context->str()<<" ]";
		for(size_t i=0;i<options.size();i++){
			if(options[i].isReady){
				cout<<" set next state = "<<options[i].id <<": "<<options[i].name<<endl;
				result = options[i].id;
				return true;
			}
		}
		return false;
	}
};

#define X(...) __VAR_ARGS__

TAO(TEST2){
	TAO_PLANS{
		A,B
	}
	TAO_START_PLAN(A);
	TAO_BGN{
		TAO_PLAN(A){
			TAO_START_CONDITION(false);
			TAO_ALLOCATE_EMPTY
			TAO_STOP_CONDITION(true);
			TAO_NEXT_EMPTY
		}
		TAO_PLAN(B){
			TAO_START_CONDITION(true);
			TAO_ALLOCATE_EMPTY
			TAO_STOP_CONDITION(true);
			TAO_NEXT_EMPTY
		}
	}
	TAO_END
}


TAO(TEST3){
	TAO_PLANS{
		A,B
	}
	TAO_START_PLAN(A);
	TAO_BGN{
		TAO_PLAN(A){
			TAO_START_CONDITION(false);
			TAO_ALLOCATE_EMPTY
			TAO_STOP_CONDITION(true);
			TAO_NEXT_EMPTY
		}
		TAO_PLAN(B){
			TAO_START_CONDITION(true);
			TAO_ALLOCATE_EMPTY
			TAO_STOP_CONDITION(true);
			TAO_NEXT_EMPTY
		}
	}
	TAO_END
}

TAO(TEST1)
{
	TAO_PLANS{
		C,
		D,
		S
	}
	TAO_START_PLAN(C);
	TAO_BGN{
		TAO_PLAN( C ){
			TAO_START_CONDITION(true);
			TAO_ALLOCATE(AllocFirstReady){
				TAO_SUBPLAN(TEST2);
				TAO_SUBPLAN(TEST3);
			}
			TAO_CALL_TASK(C);
			TAO_CLEANUP_BGN
			{
				TAO_CALL_TASK(RRR);
			}
			TAO_CLEANUP_END
			TAO_STOP_CONDITION(event == TAO_EVENT(/FAIL))
			TAO_NEXT(NextSelectFirst){
				TAO_NEXT_PLAN(D);
				TAO_NEXT_PLAN(S);
			}
		}
		TAO_PLAN( D ){
			TAO_START_CONDITION(true);
			TAO_CALL_TASK(D);
			TAO_ALLOCATE_EMPTY
			TAO_STOP_CONDITION(true);
			TAO_NEXT_EMPTY
		}
		TAO_PLAN( S ){
			TAO_START_CONDITION(true);
			TAO_RESULT("SUCCESS", TaskResult::SUCCESS());
			TAO_ALLOCATE_EMPTY
			TAO_STOP_CONDITION(true);
			TAO_NEXT_EMPTY
		}
	}
	TAO_END
}


void TAO_TEST(){
	TaskResult res = TaoTEST1(NULL,&mainEventQueue);
	cout<<"TAO_TEST: "<<res<<endl;
}

void TAO_EVENTS_GENERATOR(){
	Event spec[]={"SUCCESS","SUCCESS","SUCCESS","SUCCESS","SUCCESS","SUCCESS", "FAIL", "NOTHING"};
	int i=0;
	while(true){
		Event t = spec[i];
		if(t == "NOTHING"){ i=1; t=spec[0]; }else i++;
		cout << endl << t<<" -> ";
		mainEventQueue.riseEvent(t);
		boost::this_thread::sleep(boost::posix_time::seconds(1));
	}
}


int main() {
	boost::thread_group threads;
	threads.add_thread(new boost::thread(boost::bind(&TAO_TEST)));
	threads.add_thread(new boost::thread(boost::bind(&TAO_EVENTS_GENERATOR)));

	threads.join_all();
}

int main1() {
	TaskResult
	res = TaoTEST1_cond(NULL,&mainEventQueue, 0);
	cout<<"TAO_TEST: "<<res<<endl;
	res = TaoTEST1_cond(NULL,&mainEventQueue, 1);
	cout<<"TAO_TEST: "<<res<<endl;
	res = TaoTEST1_cond(NULL,&mainEventQueue, 2);
	cout<<"TAO_TEST: "<<res<<endl;
	return 0;
}








