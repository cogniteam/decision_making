/*
 * FSM.h
 *
 *  Created on: Nov 14, 2013
 *      Author: dan
 */

#ifndef DECISION_MAKING_FSM_H_
#define DECISION_MAKING_FSM_H_


#include "EventSystem.h"
#include "TaskResult.h"

#ifndef DMDEBUG
#define DMDEBUG(...)
#endif


namespace decision_making{

struct ScoppedThreads{
	boost::thread_group threads;
	vector<EventQueue*> events;
	void add(boost::thread* thread){threads.add_thread(thread);};
	void add(EventQueue* event){ events.push_back(event); }

	void stopEvents(){
		BOOST_FOREACH(EventQueue* e, events){
			e->close();
		}
	}
	struct Cleaner{
		ScoppedThreads& target;
		Cleaner(ScoppedThreads& target):target(target){}
		~Cleaner(){ target.stopEvents(); target.threads.join_all();}
	};
};


#define FSM_HEADER(NAME) \
	decision_making::TaskResult Fsm##NAME(const decision_making::FSMCallContext*, decision_making::EventQueue*, std::string);

#define FSM(NAME) \
	FSM_HEADER(NAME)\
	decision_making::TaskResult Fsm##NAME(const decision_making::FSMCallContext* p, decision_making::EventQueue* q){return Fsm##NAME(p,q,#NAME);}\
	decision_making::TaskResult Fsm##NAME(const decision_making::FSMCallContext* parent_call_ctx, decision_making::EventQueue* parent_event_queue, std::string fsm_name)

#define FSM_STATES enum STATES

#define __DEFCALLCONTEXT decision_making::FSMCallContext call_ctx(parent_call_ctx?decision_making::FSMCallContext(*parent_call_ctx, fsm_name):decision_making::FSMCallContext(fsm_name));
#define __DEFEVENTQUEUE decision_making::EventQueue* events_queue(parent_event_queue);
#define FSM_START(STATE) \
		state ( STATE ); \
		decision_making::TaskResult fsm_result = decision_making::TaskResult::TERMINATED();\
		__DEFCALLCONTEXT __DEFEVENTQUEUE  \
		DMDEBUG( cout<<" FSM("<<fsm_name<<":START) "; )\
		ON_FSM_START(fsm_name, call_ctx, *events_queue);

#define FSM_BGN \
		bool fsm_stop = false; \
		while(not fsm_stop and not events_queue->isTerminated()){ \
			switch(state){ { {

#define FSM_END \
			}}}} \
			DMDEBUG( cout<<" FSM("<<fsm_name<<":FINISH) "; ) \
			ON_FSM_END(fsm_name, call_ctx, *events_queue, fsm_result);\
			return fsm_result;

#define __STARTOFSTATE(X) \
		DMDEBUG( string outname("STT("+fsm_name+":"+call_ctx.str()+"/"+#X+")");cout<<outname<<"{ "; )\
		std::string state_name(#X);\
		ON_FSM_STATE_START(state_name, call_ctx, *events_queue);

#define __ENDOFSTATE \
		DMDEBUG( struct _STATE_FINISHER_PRINT{std::string n;_STATE_FINISHER_PRINT( std::string n): n(n){}~_STATE_FINISHER_PRINT(){DMDEBUG( cout<<"}"<<n<<" "; )} void r(){}}_ep(outname);_ep.r(); )\
		struct _STATE_FINISHER{\
			std::string state_name; decision_making::FSMCallContext& ctx; decision_making::EventQueue& queue;\
			_STATE_FINISHER(std::string state_name, decision_making::FSMCallContext& ctx, decision_making::EventQueue& queue): state_name(state_name),ctx(ctx),queue(queue){}\
			~_STATE_FINISHER(){\
				ON_FSM_STATE_END(state_name, ctx, queue);\
			} void r(){}\
		}_e(state_name, call_ctx, *events_queue);_e.r();

#define FSM_STATE(X)  \
			}}}break; \
			case X: { \
				ScoppedThreads SUBMACHINESTHREADS; \
				__STARTOFSTATE(X)  __ENDOFSTATE

#define FSM_NEXT(STATE) \
				state = STATE; \
				break;
#define FSM_ON_EVENT(EVENT, DO) \
			if(event==decision_making::Event(#EVENT,call_ctx)){ \
				DMDEBUG( cout<<" GOTO("<<fsm_name<<":"<<decision_making::Event(#EVENT,call_ctx)<< "->" #DO ") "; ) \
				DO;\
			}
#define FSM_ON_CONDITION(COND, DO) \
			if(COND){ \
				DMDEBUG( cout<<" GOTO("<<fsm_name<<":"<<decision_making::Event(#COND,call_ctx)<< "->" #DO ") "; ) \
				DO;\
			}


#define FSM_RISE(EVENT) \
			DMDEBUG( cout<<" RISE("<<fsm_name<<":"<<decision_making::Event(#EVENT, call_ctx)<<") "; ) \
			events_queue->riseEvent(decision_making::Event(#EVENT, call_ctx));

#define __DEFSUBEVENTQUEUE(TASK) decision_making::EventQueue events_queu##TASK(events_queue);
#define __DEFSUBCTEXT(TASK) decision_making::FSMCallContext call_ctx##TASK(call_ctx, #TASK);

#define FSM_CALL_TASK(TASK) \
			__DEFSUBEVENTQUEUE(TASK) __DEFSUBCTEXT(TASK) \
			SUBMACHINESTHREADS.add(&events_queu##TASK); \
			SUBMACHINESTHREADS.add(\
				new boost::thread(  CALL_REMOTE(TASK, boost::ref(call_ctx##TASK), boost::ref(events_queu##TASK))  ));

#define FSM_CALL_FSM(NAME) \
			__DEFSUBEVENTQUEUE(NAME) \
			SUBMACHINESTHREADS.add(&events_queu##NAME); \
			SUBMACHINESTHREADS.add(\
					new boost::thread(boost::bind(&Fsm##NAME, &call_ctx, &events_queu##NAME)  ));


#define FSM_CALL_BT(NAME) \
			__DEFSUBEVENTQUEUE(NAME) __DEFSUBCTEXT(NAME) \
			SUBMACHINESTHREADS.add(&events_queu##NAME); \
			__BT_CREATE_BT_CALL_FUNCTION(NAME)\
			SUBMACHINESTHREADS.add(\
				__CALL_BT_FUNCTION(NAME, boost::ref(call_ctx##NAME), boost::ref(events_queu##NAME))  \
			);


#define FSM_STOP(EVENT, RESULT) \
			fsm_stop=true; \
			FSM_RISE(EVENT); \
			fsm_result = RESULT; \
			break;

#define __CLEAN_THREAD_AND_EVENTS ScoppedThreads::Cleaner SUBMACHINESTHREADSCLEANER(SUBMACHINESTHREADS);
#define FSM_TRANSITIONS  __CLEAN_THREAD_AND_EVENTS {Event event; while((event=events_queue->waitEvent())==true){

#define FSM_DROP_EVENTS events_queue->drop_all();

#define FSM_PRINT_EVENT cout<<" READ("<<fsm_name<<":"<<event<<") ";

}


#endif /* FSM_H_ */
