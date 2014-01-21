/*
 * TAO.h
 *
 *  Created on: Nov 14, 2013
 *      Author: dan
 */

#ifndef DECISION_MAKING_TAO_H_
#define DECISION_MAKING_TAO_H_


#include "EventSystem.h"
#include "TaskResult.h"

#ifndef DMDEBUG
#define DMDEBUG(...)
#endif

#include <boost/function.hpp>
#include <boost/bind.hpp>

namespace decision_making{

class ___ABS__ScoppedThreadsOnExit{
public:
    virtual ~___ABS__ScoppedThreadsOnExit(){}
    virtual void exit()=0;
    virtual boost::thread_group& getThreads()=0;
};
struct ScoppedThreads{
	typedef boost::shared_ptr<EventQueue> EventQueuePtr;
	typedef boost::shared_ptr<CallContext> CallContextPtr;
	typedef boost::shared_ptr<___ABS__ScoppedThreadsOnExit> ScoppedThreadsOnExitPtr;
	boost::thread_group threads;
	vector<EventQueuePtr> events;
	vector<CallContextPtr> contexts;
	void add(boost::thread* thread){threads.add_thread(thread);};
	void add(EventQueuePtr event){ events.push_back(event); }
	void add(CallContextPtr event){ contexts.push_back(event); }

	void stopEvents(){
		BOOST_FOREACH(EventQueuePtr e, events){
			e->close();
		}
	}
	struct Cleaner{
		ScoppedThreads& target;
		Cleaner(ScoppedThreads& target):target(target){}
		~Cleaner(){
			target.runOnExit();
			target.stopEvents();
			target.threads.join_all();
		}
	};

	vector<ScoppedThreadsOnExitPtr> on_exits;
	void add(ScoppedThreadsOnExitPtr exit){ on_exits.push_back(exit); }
	//void runOnExit();
	void runOnExit(){
	    BOOST_FOREACH(ScoppedThreadsOnExitPtr e, on_exits){
	        e->exit();
	        e->getThreads().join_all();
	    }
	}
};
class ScoppedThreadsOnExit:public ___ABS__ScoppedThreadsOnExit{
public:
	EventQueue* events_queue;
	CallContext& plan_call_ctx;
	ScoppedThreads SUBMACHINESTHREADS;
	ScoppedThreadsOnExit(CallContext& call_ctx, EventQueue* events_queue):
		events_queue(events_queue), plan_call_ctx(call_ctx)
	{}
	virtual ~ScoppedThreadsOnExit(){}
	//virtual void exit()=0;
	virtual boost::thread_group& getThreads(){ return SUBMACHINESTHREADS.threads; }
};

#define _TAO_FUN_SIG boost::function<decision_making::TaskResult(int)>
class TaoConditionCallback{public:
	_TAO_FUN_SIG cond;
	virtual ~TaoConditionCallback(){};
};

class ProtocolNext{
protected:
	int& result;
	decision_making::CallContext* call_context;
	decision_making::EventQueue* events;
	struct Option{
		int id;
		std::string name;
		bool isReady;
		Option(int id, std::string name, bool isReady):id(id),name(name),isReady(isReady){}
	};
	std::vector<Option> options;
public:
	ProtocolNext(int& res, decision_making::CallContext* call_context, decision_making::EventQueue* events):
		result(res), call_context(call_context), events(events){}
	virtual ~ProtocolNext(){ }
	void add(int id, std::string name, bool isReady){ options.push_back(Option(id, name, isReady)); }
	virtual bool decide()=0;
	void resetResult(){ result = -1; }

	struct Cleanner{
		ProtocolNext& p;
		Cleanner(ProtocolNext& p):p(p){}
		~Cleanner(){ if(not p.decide()) p.resetResult(); }
	};
};

class ProtocolAllocation{
public:
	typedef boost::function<decision_making::TaskResult(void)> Callback;
protected:
	int& result;
	decision_making::CallContext* call_context;
	decision_making::EventQueue* events;
	int opt_counter;
	struct Option{
		Callback callback;
		int id;
		std::string name;
		bool isReady;
		Option(int id, std::string name, bool isReady, Callback cb):callback(cb),id(id),name(name),isReady(isReady){}
	};
	std::vector<Option> options;
public:
	ProtocolAllocation(int& res, decision_making::CallContext* call_context, decision_making::EventQueue* events):
		result(res), call_context(call_context), events(events), opt_counter(0){}
	virtual ~ProtocolAllocation(){ }
	void add(std::string name, bool isReady, Callback cb){ options.push_back(Option(opt_counter++, name, isReady, cb)); }
	virtual bool decide()=0;
	Callback getCallback()const{
		return options.at(result).callback;
	}

	struct Cleanner{
		ProtocolAllocation& p;
		ScoppedThreads& SUBMACHINESTHREADS;
		decision_making::ScoppedThreads::EventQueuePtr events;
		Cleanner(ProtocolAllocation& p, ScoppedThreads& SUBMACHINESTHREADS, decision_making::ScoppedThreads::EventQueuePtr events):p(p),SUBMACHINESTHREADS(SUBMACHINESTHREADS), events(events){}
		~Cleanner(){
			//cout<<"[ProtocolAllocation::Cleanner]";
			if( p.decide() ){
				SUBMACHINESTHREADS.add(events);
				SUBMACHINESTHREADS.add(new boost::thread( p.getCallback() ));
			}
		}
	};
};

class _NextPEMPTY:public decision_making::ProtocolNext{
public:
	_NextPEMPTY(int& res, decision_making::CallContext* call_context, decision_making::EventQueue* events):ProtocolNext(res, call_context, events){}
	bool decide(){
		result = 0;
		return false;
	}
};
class _AllocPEMPTY:public decision_making::ProtocolAllocation{
public:
	_AllocPEMPTY(int& res, decision_making::CallContext* call_context, decision_making::EventQueue* events):ProtocolAllocation(res, call_context, events){}
	bool decide(){
		result = 0;
		return false;
	}
};


#define TAO_HEADER(NAME) \
	decision_making::TaskResult Tao##NAME(const decision_making::CallContext*, decision_making::EventQueue*, std::string, bool jc, int tss, TaoConditionCallback*);\
	decision_making::TaskResult Tao##NAME##_cond(const decision_making::CallContext* p, decision_making::EventQueue* q, int tss);\
	decision_making::TaskResult Tao##NAME(const decision_making::CallContext* p, decision_making::EventQueue* q);

#define TAO(NAME) \
	TAO_HEADER(NAME)\
	decision_making::TaskResult Tao##NAME##_cond(const decision_making::CallContext* p, decision_making::EventQueue* q, int tss){\
				class Cond: public TaoConditionCallback{ } f;\
				return Tao##NAME(p,q,#NAME, true, tss, &f);\
			}\
	decision_making::TaskResult Tao##NAME(const decision_making::CallContext* p, decision_making::EventQueue* q){\
				class Cond: public TaoConditionCallback{ } f;\
				f.cond = boost::bind(&Tao##NAME##_cond, p, q, _1);\
				return Tao##NAME(p,q,#NAME, false, -1, &f);\
			}\
	decision_making::TaskResult Tao##NAME(\
			const decision_making::CallContext* parent_call_ctx,\
			decision_making::EventQueue* parent_event_queue,\
			std::string tao_name,\
			bool justCondition, int try_start_state, TaoConditionCallback* this_function)

#define TAO_PLANS enum TAOPLANS

#define __DEFCALLCONTEXT decision_making::CallContext call_ctx(parent_call_ctx?decision_making::CallContext(*parent_call_ctx, tao_name):decision_making::CallContext(tao_name));
#define __DEFEVENTQUEUE decision_making::EventQueue* events_queue(parent_event_queue);
#define __TAO_FINISHER \
		struct _tao_finisher_type{\
			bool run;\
			const decision_making::CallContext* parent_call_ctx;\
			decision_making::EventQueue* parent_event_queue;\
			_tao_finisher_type(bool run, const decision_making::CallContext* parent_call_ctx, decision_making::EventQueue* parent_event_queue):\
				run(run),parent_call_ctx(parent_call_ctx), parent_event_queue(parent_event_queue){}\
			~_tao_finisher_type(){\
					if(!run or !parent_event_queue or !parent_call_ctx)return; \
					parent_event_queue->raiseEvent(decision_making::Event("SUBPLAN_FINISHED",*parent_call_ctx));\
			}\
		} _tao_finisher(!justCondition, parent_call_ctx, parent_event_queue);\

#define TAO_START_PLAN(STATE) \
		; int state ( try_start_state<0? (int)STATE : try_start_state ); \
		decision_making::TaskResult tao_result = decision_making::TaskResult::TERMINATED();\
		__DEFCALLCONTEXT __DEFEVENTQUEUE __TAO_FINISHER\
		if(!justCondition){\
			DMDEBUG( cout<<" TAO("<<tao_name<<":START) "; )\
			ON_TAO_START(tao_name, call_ctx, *events_queue);\
		}

#define TAO_BGN \
		bool tao_stop = false; \
		while(not tao_stop and not events_queue->isTerminated() DM_SYSTEM_STOP){ \
			switch((TAOPLANS)state){ { {

#define TAO_END \
			}}break;\
			default: tao_stop=true; tao_result = decision_making::TaskResult::FAIL("SELECTED BEH DOES NOT EXIST") ;  break;}} \
			if(!justCondition){\
				DMDEBUG( cout<<" TAO("<<tao_name<<":FINISH) "; ) \
				ON_TAO_END(tao_name, call_ctx, *events_queue, tao_result);\
			}\
			return tao_result;

#define __STARTOFSTATE(X) \
		std::string plan_name(#X);\
		decision_making::CallContext plan_call_ctx(call_ctx, plan_name);\
		if(!justCondition){\
			DMDEBUG( string outname("STT("+tao_name+":"+call_ctx.str()+"/"+#X+")");cout<<outname<<"{ "; )\
			ON_TAO_STATE_START(plan_name, call_ctx, *events_queue);\
		}

#define __ENDOFSTATE \
		DMDEBUG( struct _STATE_FINISHER_PRINT{std::string n; bool jc; _STATE_FINISHER_PRINT( std::string n, bool jc): n(n),jc(jc){}~_STATE_FINISHER_PRINT(){if(jc)return;DMDEBUG( cout<<"}"<<n<<" "; )} void r(){}}_ep(outname,justCondition);_ep.r(); )\
		struct _STATE_FINISHER{\
			std::string plan_name; decision_making::CallContext& ctx; decision_making::EventQueue& queue; bool jc;\
			_STATE_FINISHER(std::string plan_name, decision_making::CallContext& ctx, decision_making::EventQueue& queue, bool jc): plan_name(plan_name),ctx(ctx),queue(queue),jc(jc){}\
			~_STATE_FINISHER(){\
				if(!jc){ ON_TAO_STATE_END(plan_name, ctx, queue); }\
			} void r(){}\
		}_e(plan_name, call_ctx, *events_queue, justCondition);_e.r();

#define TAO_CONTEXT plan_call_ctx

#define TAO_PLAN(X)  \
			}}}break; \
			case X: { \
				decision_making::ScoppedThreads SUBMACHINESTHREADS; \
				__STARTOFSTATE(X)  __ENDOFSTATE

#define TAO_GO_NEXT(STATE) \
				state = STATE; \
				break;
#define TAO_ON_EVENT(EVENT, DO) \
			if(event==decision_making::Event(EVENT,plan_call_ctx)){ \
				DMDEBUG( cout<<" GOTO("<<tao_name<<":"<<decision_making::Event(EVENT,plan_call_ctx)<< "->" #DO ") "; ) \
				DO;\
			}
#define TAO_EVENT(EVENT) decision_making::Event(#EVENT,plan_call_ctx)

#define TAO_ON_CONDITION(COND, DO) \
			if(COND){ \
				DMDEBUG( cout<<" GOTO("<<tao_name<<":"<<decision_making::Event(#COND,plan_call_ctx)<< "->" #DO ") "; ) \
				DO;\
			}

#define TAO_RAISE(EVENT) \
			DMDEBUG( cout<<" RAISE("<<tao_name<<":"<<decision_making::Event(EVENT, plan_call_ctx)<<") "; ) \
			events_queue->raiseEvent(decision_making::Event(EVENT, plan_call_ctx));

//Deprecated
#define TAO_RISE(EVENT) TAO_RAISE(EVENT)

#define TAO_EVENTS_DROP events_queue->drop_all();

#define __DEFSUBEVENTQUEUE(TASK) decision_making::ScoppedThreads::EventQueuePtr events_queu##TASK( new decision_making::EventQueue(events_queue) );
#define __DEFSUBCTEXT(TASK) decision_making::ScoppedThreads::CallContextPtr call_ctx##TASK( new decision_making::CallContext(plan_call_ctx, #TASK) );
#define __SHR_TO_REF(X) (*(X.get()))
#define TAO_CALL_TASK(TASK) \
			__DEFSUBEVENTQUEUE(TASK) __DEFSUBCTEXT(TASK) \
			SUBMACHINESTHREADS.add(events_queu##TASK); \
			SUBMACHINESTHREADS.add(call_ctx##TASK); \
			SUBMACHINESTHREADS.add(\
				new boost::thread(  CALL_REMOTE(TASK, boost::ref(__SHR_TO_REF(call_ctx##TASK)), boost::ref(__SHR_TO_REF(events_queu##TASK)))  ));

#define TAO_CALL_FSM(NAME) \
			__DEFSUBEVENTQUEUE(NAME) \
			SUBMACHINESTHREADS.add(events_queu##NAME); \
			SUBMACHINESTHREADS.add(\
					new boost::thread(boost::bind(&Fsm##NAME, &call_ctx, events_queu##NAME.get())  ));
#define TAO_CALL_TAO(NAME) \
			__DEFSUBEVENTQUEUE(NAME) \
			SUBMACHINESTHREADS.add(events_queu##NAME); \
			SUBMACHINESTHREADS.add(\
					new boost::thread(boost::bind(&Tao##NAME, &call_ctx, events_queu##NAME.get())  ));


#define TAO_CALL_BT(NAME) \
			__DEFSUBEVENTQUEUE(NAME) __DEFSUBCTEXT(NAME) \
			SUBMACHINESTHREADS.add(events_queu##NAME); \
			SUBMACHINESTHREADS.add(call_ctx##NAME); \
			decision_making::EventQueue& __t_events_queu##NAME = __SHR_TO_REF(events_queu##NAME);\
			decision_making::CallContext& __t_call_ctx##NAME = __SHR_TO_REF(call_ctx##NAME);\
			__BT_CREATE_BT_CALL_FUNCTION(NAME, __t_call_ctx##NAME, __t_events_queu##NAME)\
			SUBMACHINESTHREADS.add(\
				__CALL_BT_FUNCTION(NAME, boost::ref(__t_call_ctx##NAME), boost::ref(__t_events_queu##NAME))  \
			);

#define TAO_CLEANUP_BGN \
			class __ON_STATE_EXIT_STRUCT:public decision_making::ScoppedThreadsOnExit{public:\
				__ON_STATE_EXIT_STRUCT(CallContext& call_ctx, EventQueue* events_queue):decision_making::ScoppedThreadsOnExit(call_ctx, events_queue){}\
				virtual void exit(){

#define TAO_CLEANUP_END \
				}\
			};\
			decision_making::ScoppedThreadsOnExit* __tmp___ON_STATE_EXIT_STRUCT = new __ON_STATE_EXIT_STRUCT(call_ctx, events_queue);\
			SUBMACHINESTHREADS.add(decision_making::ScoppedThreads::ScoppedThreadsOnExitPtr(__tmp___ON_STATE_EXIT_STRUCT));

#define TAO_RESULT(EVENT, RESULT) \
			tao_stop=true; \
			TAO_RAISE(EVENT); \
			tao_result = RESULT; \
			break;

#define TAO_START_CONDITION(CONDITION) \
		if(justCondition){\
			tao_stop=true; \
			bool _l_cond_satis = (CONDITION);\
			tao_result = _l_cond_satis ? decision_making::TaskResult::SUCCESS() : decision_making::TaskResult::FAIL(#CONDITION" is False") ; \
			return tao_result;\
		}

#define __CLEAN_THREAD_AND_EVENTS decision_making::ScoppedThreads::Cleaner SUBMACHINESTHREADSCLEANER(SUBMACHINESTHREADS);
#define TAO_TRANSITIONS  __CLEAN_THREAD_AND_EVENTS {Event event; while((event=events_queue->waitEvent())==true){

#define TAO_NEXT(ProtocolName)  }} {{\
			ProtocolName protocol(state,&plan_call_ctx, events_queue); \
			decision_making::ProtocolNext::Cleanner cleanner(protocol);

#define TAO_NEXT_EMPTY TAO_NEXT(_NextPEMPTY)

#define TAO_NEXT_PLAN(BEHNAME) protocol.add((int)BEHNAME, #BEHNAME, this_function->cond(BEHNAME).isSuccess());


#define TAO_ALLOCATE(ProtocolName)  {\
			ProtocolName protocol(state,&plan_call_ctx, events_queue); \
			__DEFSUBEVENTQUEUE(ALLOC)\
			decision_making::ProtocolAllocation::Cleanner cleanner(protocol, SUBMACHINESTHREADS ,events_queu##ALLOC);

#define TAO_ALLOCATE_EMPTY TAO_ALLOCATE(_AllocPEMPTY)


#define TAO_SUBPLAN(TAONAME) \
		DMDEBUG( cout<<"[ADD ROLE: "<<#TAONAME<<"]"; )\
		protocol.add(#TAONAME, Tao##TAONAME##_cond(&plan_call_ctx, events_queue, -1).isSuccess(), boost::bind(&Tao##TAONAME, &plan_call_ctx, events_queu##ALLOC.get()));

#define TAO_STOP_CONDITION(COND)  \
			}\
			TAO_TRANSITIONS\
			{\
				if(event==TAO_EVENT(SUBPLAN_FINISHED)){ break; }\
				if(COND){ break ;}\
			}

#define TAO_STOP_CONDITION_AND_PRINT_EVENTS(COND)  \
			}\
			TAO_TRANSITIONS\
			{\
				TAO_PRINT_EVENT\
				if(event==TAO_EVENT(SUBPLAN_FINISHED)){ break; }\
				if(COND){ break ;}\
			}

#define TAO_DROP_EVENTS events_queue->drop_all();

#define TAO_PRINT_EVENT if(not event.equals(decision_making::Event::SPIN_EVENT())){ cout<<" READ("<<tao_name<<":"<<event<<") "; }

}


#endif /* TAO_H_ */
