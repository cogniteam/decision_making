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
#include "FSM.h"
using namespace decision_making;

EventQueue mainEventQueue;


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
		}
	}
}

#define CALL_REMOTE(NAME, CALLS, EVENTS) boost::bind(&callTask, #NAME, CALLS, EVENTS)


#define X(...) __VA_ARGS__

FSM(TEST1){
	enum STATS{
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
//				FAM_NEXT(D/SUCCESS, FSM_NEXT(D));
//				FAM_NEXT(D/FAIL, FSM_NEXT(D));
			}
		}
		FSM_STATE( S ){
			FSM_STOP(SUCCESS, TaskResult::SUCCESS());
			FSM_TRANSITIONS{}
		}
	}
	FSM_END
}
//X(
FSM(TEST){
	enum STATS{
		A,
		B,
		STOP
	}
	FSM_START(A);
	FSM_BGN{
		FSM_STATE( A ){
			FSM_CALL_TASK(TA);
			FSM_CALL_TASK(TB);
			FSM_RISE(GO_TO_STOP)
			FSM_TRANSITIONS{
				FSM_PRINT_EVENT;
				FSM_ON_EVENT(TA/GO, FSM_NEXT(B));
				FSM_ON_EVENT(/A/SUCCESS, FSM_NEXT(B));
				//FSM_ON_EVENT(/A/FAIL, FSM_NEXT(B));
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

void FSM_EVENTS_GENERATOR(){
	Event spec[]={"SUCCESS", "FAIL", "SUCCESS", "FAIL", "SUCCESS", "FAIL", "SUCCESS", "FAIL", "GO", "NOTHING"};
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
threads.add_thread(new boost::thread(boost::bind(&FSM_TEST)));
threads.add_thread(new boost::thread(boost::bind(&FSM_EVENTS_GENERATOR)));

threads.join_all();
}






//-------------------------------------------------------------



//FSM(Turnstile)
//{
//	enum STATES
//	{
//		Locked,
//		Unlocked
//	}
//	FSM_START(Locked);
//	FSM_BGN
//	{
//		FSM_STATE(Locked)
//		{
//			FSM_TRANSITIONS
//			{
//				FSM_ON_EVENT(COIN, FSM_NEXT(Unlocked));
//			}
//		}
//		FSM_STATE(Unlocked)
//		{
//			FSM_TRANSITIONS
//			{
//				FSM_ON_EVENT(PUSH, FSM_NEXT(Locked));
//			}
//		}
//	}
//	FSM_END
//}
//
//bool leftBumper, rightBumper, wallSensor;
//FSM(Roomba)
//{
//	enum STATES
//	{
//		findWall,
//		turnSx,
//		correctSx,
//		correctDx
//	}
//	FSM_START(findWall);
//	FSM_BGN
//	{
//		FSM_STATE(findWall)
//		{
//			FSM_TRANSITIONS
//			{
//				FSM_ON_CONDITION( leftBumper==1||rightBumper==1 , FSM_NEXT(turnSx));
//			}
//		}
//		FSM_STATE(turnSx)
//		{
//
//			FSM_TRANSITIONS
//			{
//				FSM_ON_CONDITION( leftBumper==0&&rightBumper==0 , FSM_NEXT(correctDx));
//			}
//		}
//		FSM_STATE(correctSx)
//		{
//
//			FSM_TRANSITIONS
//			{
//				FSM_ON_CONDITION( leftBumper==1||rightBumper==1 , FSM_NEXT(turnSx));
//				FSM_ON_CONDITION( wallSensor==0 , FSM_NEXT(correctDx));
//			}
//		}
//		FSM_STATE(correctDx)
//		{
//
//			FSM_TRANSITIONS
//			{
//				FSM_ON_CONDITION( leftBumper==1||rightBumper==1 , FSM_NEXT(turnSx));
//				FSM_ON_CONDITION( wallSensor==1 , FSM_NEXT(correctSx));
//			}
//		}
//	}
//	FSM_END
//}
////-------------------- HIERARCHY ------------------------
//
//FSM(Superstate)
//{
//	enum STATES
//	{
//		Start, Process
//	}
//	FSM_START(Start);
//	FSM_BGN
//	{
//		FSM_STATE(Start)
//		{
//			FSM_TRANSITIONS
//			{
//				FSM_ON_EVENT(/begin, FSM_NEXT(Process));
//				FSM_ON_EVENT(/end, FSM_RISE(Start/end));
//				FSM_ON_EVENT(/data, FSM_RISE(Start/data));
//			}
//		}
//		FSM_STATE(Process)
//		{
//			FSM_TRANSITIONS
//			{
//				FSM_ON_EVENT(/data, FSM_NEXT(Process));
//				FSM_ON_EVENT(/end, FSM_RISE(Process/end));
//				FSM_ON_EVENT(/data, FSM_RISE(Process/data));
//			}
//		}
//	}
//	FSM_END
//}
//
//FSM(ScxmlExampleHierarchy)
//{
//	enum STATES
//	{
//		Success, Error, Superstate
//	}
//	FSM_START(Superstate);
//	FSM_BGN
//	{
//		FSM_STATE(Success)
//		{
//			FSM_TRANSITIONS
//		}
//		FSM_STATE(Error)
//		{
//			FSM_TRANSITIONS
//		}
//		FSM_STATE(Superstate)
//		{
//			FSM_CALL_FSM(Superstate);
//			FSM_TRANSITIONS
//			{
//				FSM_ON_EVENT(/error, FSM_NEXT(Error));
//				FSM_ON_EVENT(Superstate/error, FSM_NEXT(Error));
//				FSM_ON_EVENT(Superstate/Start/end, FSM_NEXT(Error));
//				FSM_ON_EVENT(Superstate/Start/data, FSM_NEXT(Error));
//				FSM_ON_EVENT(Superstate/Process/begin, FSM_NEXT(Error));
//				FSM_ON_EVENT(Superstate/Process/end, FSM_NEXT(Success));
//			}
//		}
//
//	}
//	FSM_END
//}
////------------------------ PARALLEL -----------------------
//FSM(A)
//{
//	enum STATES
//	{
//		a0, a1
//	}
//	FSM_START(a0);
//	FSM_BGN
//	{
//		FSM_STATE(a0)
//		{
//			FSM_TRANSITIONS
//			{
//				FSM_ON_EVENT(/a, FSM_NEXT(a1));
//				FSM_ON_EVENT(/d, FSM_RISE(a1/d));
//			}
//		}
//		FSM_STATE(a1)
//		{
//			FSM_TRANSITIONS
//			{
//				FSM_ON_EVENT(/a, FSM_RISE(a1/a));
//				FSM_ON_EVENT(/d, FSM_RISE(d));
//			}
//		}
//	}
//	FSM_END
//}
//FSM(B)
//{
//	enum STATES
//	{
//		b0, b1
//	}
//	FSM_START(b0);
//	FSM_BGN
//	{
//		FSM_STATE(b0)
//		{
//			FSM_TRANSITIONS
//			{
//				FSM_ON_EVENT(/b, FSM_NEXT(b1));
//				FSM_ON_EVENT(/d, FSM_RISE(b1/d));
//			}
//		}
//		FSM_STATE(b1)
//		{
//			FSM_TRANSITIONS
//			{
//				FSM_ON_EVENT(/b, FSM_RISE(b1/b));
//				FSM_ON_EVENT(/d, FSM_RISE(d));
//			}
//		}
//	}
//	FSM_END
//}
//FSM(C)
//{
//	enum STATES
//	{
//		c0, c1
//	}
//	FSM_START(c0);
//	FSM_BGN
//	{
//		FSM_STATE(c0)
//		{
//			FSM_TRANSITIONS
//			{
//				FSM_ON_EVENT(/c, FSM_NEXT(c1));
//				FSM_ON_EVENT(/d, FSM_RISE(c1/d));
//			}
//		}
//		FSM_STATE(c1)
//		{
//			FSM_TRANSITIONS
//			{
//				FSM_ON_EVENT(/c, FSM_RISE(c1/c));
//				FSM_ON_EVENT(/d, FSM_RISE(d));
//			}
//		}
//	}
//	FSM_END
//}
//
//FSM(ScxmlExampleParallel)
//{
//	enum STATES
//	{
//		Success, Error, Parallel
//	}
//	FSM_START(Parallel);
//	FSM_BGN
//	{
//		FSM_STATE(Success)
//		{
//			FSM_TRANSITIONS
//		}
//		FSM_STATE(Error)
//		{
//			FSM_TRANSITIONS
//		}
//		FSM_STATE(Parallel)
//		{
//			int finished_counter=0;
//			FSM_CALL_FSM(A);
//			FSM_CALL_FSM(B);
//			FSM_CALL_FSM(C);
//			FSM_TRANSITIONS
//			{
//				FSM_ON_EVENT(A/a0/d, FSM_NEXT(Error));
//				FSM_ON_EVENT(B/b0/d, FSM_NEXT(Error));
//				FSM_ON_EVENT(C/c0/d, FSM_NEXT(Error));
//				FSM_ON_EVENT(A/a1/a, FSM_NEXT(Error));
//				FSM_ON_EVENT(B/b1/b, FSM_NEXT(Error));
//				FSM_ON_EVENT(C/c1/c, FSM_NEXT(Error));
//				FSM_ON_EVENT(A/d, finished_counter++ );
//				FSM_ON_EVENT(B/d, finished_counter++ );
//				FSM_ON_EVENT(C/d, finished_counter++ );
//				FSM_ON_CONDITION( finished_counter==3 , FSM_RISE(d));
//				FSM_ON_EVENT( d , FSM_NEXT(Success));
//				//FSM_ON_CONDITION( finished_counter==3 , FSM_NEXT(Success));
//			}
//		}
//	}
//	FSM_END
//}








