/*
 * BT.h
 *
 *  Created on: Nov 5, 2013
 *      Author: dan
 */

#ifndef BT_H_
#define BT_H_

#include <boost/thread.hpp>
#include <deque>
#include <vector>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>
#include <boost/shared_ptr.hpp>

#include "EventSystem.h"
#include "TaskResult.h"

#ifndef DMDEBUG
#define DMDEBUG(...)
#endif

namespace decision_making{

struct BTContext{};
struct BTNode{
	enum BTNodeTYPE{ BT_SEQ, BT_SEL, BT_PAR, BT_TASK, BT_PAR_SEQ, BT_PAR_SEL };
	BTNodeTYPE type;
	boost::thread_group threads;
	std::vector< BTNode* > tasks;
	boost::mutex m;
	bool terminated;
	bool finished;
	boost::condition_variable on_child_terminated;
	TaskResult bt_node_return_value;
	CallContext call_ctx;
	EventQueue events;
	string node_name;
	#define BTNodeContructorParams_DEF BTNodeTYPE t, string node_name, const CallContext& parent_call_ctx, EventQueue& events
	#define BTNodeContructorParams_USE parent_call_ctx, events
	BTNode(BTNodeContructorParams_DEF):
		type(t),
		terminated(false),
		finished(false),
		bt_node_return_value(TaskResult::UNDEF()),
		node_name(node_name),
		call_ctx(parent_call_ctx, node_name),
		events(&events, true)
	{}
	virtual ~BTNode(){}
	virtual TaskResult run(){return TaskResult::UNDEF();};
	void run_task(BTNode* t){
		TaskResult ret = run();
		//cout<<"  F:"<<ret<<endl;
		{boost::mutex::scoped_lock l(t->m);
			if(t->bt_node_return_value.isUndefined() || t->type!=BT_PAR){
				t->bt_node_return_value = ret;
			}
			t->on_child_terminated.notify_one();
		}
	};
	void run_thread(BTNode* BTNode){
		threads.add_thread(new boost::thread (boost::bind(&BTNode::run_task,BTNode, this)));
	}
	void join(){ threads.join_all(); }
	void run_all(){
		if(type==BT_PAR){
			for(size_t i=0;i<tasks.size();i++){
				tasks[i]->bt_node_return_value = bt_node_return_value;
				run_thread(tasks[i]);
			}
			{boost::mutex::scoped_lock l(m);
				on_child_terminated.wait(l);
				_us_terminate();
			}
			DMDEBUG( cout<<"[par: someone finished]"; )
			join();
		}else
		if(type==BT_SEQ){
			for(size_t i=0;i<tasks.size() && isTerminated()==false && not bt_node_return_value.isFail();i++){
				tasks[i]->bt_node_return_value = bt_node_return_value;
				bt_node_return_value = tasks[i]->run();
			}
		}else
		if(type==BT_SEL){
			for(size_t i=0;i<tasks.size() && isTerminated()==false && not bt_node_return_value.isSuccess();i++){
				tasks[i]->bt_node_return_value = bt_node_return_value;
				bt_node_return_value = tasks[i]->run();
			}
		}else
		if(type==BT_TASK){

		}else
		if(type==BT_PAR_SEQ){
			size_t tasks_count = tasks.size();
			for(size_t i=0;i<tasks.size();i++){
				tasks[i]->bt_node_return_value = bt_node_return_value;
				run_thread(tasks[i]);
			}
			TaskResult _ret = TaskResult::UNDEF();
			{boost::mutex::scoped_lock l(m);
				on_child_terminated.wait(l);
				tasks_count--;
				if(bt_node_return_value.isFail() || tasks_count==0){
					_ret = bt_node_return_value;
					_us_terminate();
				}
			}
			join();
			bt_node_return_value = _ret;
		}else
		if(type==BT_PAR_SEL){
			size_t tasks_count = tasks.size();
			for(size_t i=0;i<tasks.size();i++){
				tasks[i]->bt_node_return_value = bt_node_return_value;
				run_thread(tasks[i]);
			}
			TaskResult _ret = TaskResult::UNDEF();
			{boost::mutex::scoped_lock l(m);
				on_child_terminated.wait(l);
				tasks_count--;
				if(bt_node_return_value.isSuccess() || tasks_count==0)
					_ret = bt_node_return_value;
					_us_terminate();
			}
			join();
			bt_node_return_value = _ret;
		}
		{boost::mutex::scoped_lock l(m);
			finished = true;
		}
	}
	void _us_terminate(){
		if(finished) return;
		terminated = true;
		events.close();
		for(size_t i=0;i<tasks.size();i++)tasks[i]->terminate();
	}
	void terminate(){
		boost::mutex::scoped_lock l(m);
		_us_terminate();
	}
	bool isTerminated(){
		boost::mutex::scoped_lock l(m);
		return terminated;
	}
};

static EventQueue& __tmp_event_queue(){static EventQueue tmp; return tmp; };

typedef BTNode CurrentNodeType;

#define BT_TASK_RESULT(X) bt_node_return_value = X
#define BT_NODE(NAME) __BT_NODE_##NAME##_INSTANCE
#define BT_NODE_TYPE(NAME) __BT_NODE_##NAME##_STRUCT
#define BT_NODE_PTR(NAME) boost::shared_ptr<BTNode>
#define BT_NODE_NEW_PTR(NAME) BT_NODE_PTR(NAME)((BTNode*)new BT_NODE_TYPE(NAME)(this))
#define BT_LAST_NODE __ALL_NODES.back()

#define BT_RUN_LAST_NODE BT_LAST_NODE->run()
#define BT_RUN_NODE(NAME) BT_NODE(NODE)->run()

#define BT_RENAME_CONTEXT(NEW_NAME) typedef BTContext NEW_NAME##Type; NEW_NAME##Type& NEW_NAME=context;
#define BT_NEW_CONTEXT(...) struct BTContext{__VA_ARGS__} context

#define BT_CALL_BT(NAME) \
		BT_NODE_PTR(NAME) BT_NODE(NAME)((BTNode*)new BT_NODE_TYPE(NAME)(this, context, call_ctx, events));\
		__ALL_NODES.push_back(BT_NODE(NAME))

#define __BT_CALL_BT_CALLER(NAME) \
		call_ctx.pop(); \
		BT_NODE_PTR(NAME) BT_NODE(NAME)((BTNode*)new BT_NODE_TYPE(NAME)(this, context, call_ctx, events));\
		__ALL_NODES.push_back(BT_NODE(NAME))

#define BT_HEADER(NAME) struct BT_NODE_TYPE(NAME);

#define BT_ROOT_BGN(NAME,EVENTS)\
	struct BT_NODE_TYPE(NAME):public BTNode\
	{ \
		BTContext _tmp_context; BTContext& context;\
		BT_NODE_TYPE(NAME)():BTNode(BT_SEQ, #NAME, decision_making::CallContext(), EVENTS),_tmp_context(),context(_tmp_context){}\
		BT_NODE_TYPE(NAME)(BTContext& ctx, const decision_making::CallContext& calls, decision_making::EventQueue& events):BTNode(BT_SEQ, #NAME, calls, events),context(ctx){}\
		TaskResult run(){\
			DMDEBUG( cout<<" [BT:" #NAME " MAIN" "]{ "; ) \
			ON_BT_NODE_START(#NAME, "ROOT", call_ctx, events);\
			BTNode* const& selfPtrForRosTaskCaller=this;\
			std::vector< BT_NODE_PTR() > __ALL_NODES;

#define __BT_NODE_BGN(TYPE, NAME, STR)\
	struct BT_NODE_TYPE(NAME):public BTNode\
	{ \
		BTContext& context;\
		BT_NODE_TYPE(NAME)(BTNode* p, BTContext& ctx, const decision_making::CallContext& calls, decision_making::EventQueue& events):BTNode(TYPE, #NAME, calls, events),context(ctx){\
			p->tasks.push_back(this);\
		}\
		TaskResult run()\
		{\
			DMDEBUG( cout<<" [BT:" #NAME STR "]{ "; ) \
			ON_BT_NODE_START(#NAME, #TYPE, call_ctx, events);\
			BTNode* const& selfPtrForRosTaskCaller=this;\
			std::vector<BT_NODE_PTR()> __ALL_NODES;

#define __BT_END_NODE(NAME)\
			this->run_all();\
			DMDEBUG( cout<<" }[BT:" #NAME "] "; )\
			ON_BT_NODE_END(#NAME, call_ctx, events, bt_node_return_value);\
			return bt_node_return_value;\
		}\
	}

#define BT_BGN(NAME) __BT_NODE_BGN(BT_SEQ, NAME, "=>")
#define BT_PAR_BGN(NAME)  __BT_NODE_BGN(BT_PAR, NAME, "||")
#define BT_SEQ_BGN(NAME)  __BT_NODE_BGN(BT_SEQ, NAME, "->")
#define BT_SEL_BGN(NAME)  __BT_NODE_BGN(BT_SEL, NAME, "??")
#define BT_TASK_BGN(NAME) __BT_NODE_BGN(BT_TASK, NAME, "") decision_making::TaskResult bt_node_return_value = decision_making::TaskResult::UNDEF();

#define BT_END(NAME) __BT_END_NODE(NAME)

#define __BT_END_TASK(NAME) __BT_END_NODE(NAME); BT_CALL_BT(NAME)

#define BT_PAR_END(NAME)  __BT_END_TASK(NAME)
#define BT_SEQ_END(NAME)  __BT_END_TASK(NAME)
#define BT_SEL_END(NAME)  __BT_END_TASK(NAME)
#define BT_TASK_END(NAME) __BT_END_TASK(NAME)

#define __BTDEFSUBEVENTQUEUE(NAME) decision_making::EventQueue events_##NAME(&events);
//#define __BTDEFSUBCTEXT(NAME) CallContext call_ctx_##NAME(call_ctx, #NAME);
//#define __BTDEFSUBEVENTQUEUE(NAME) EventQueue& events_##NAME(events);
#define __BTDEFSUBCTEXT(NAME) decision_making::CallContext& call_ctx_##NAME(call_ctx);

#define BT_CALL_TASK(NAME)\
					__BT_NODE_BGN(BT_TASK, NAME, "")\
						__BTDEFSUBEVENTQUEUE(NAME) __BTDEFSUBCTEXT(NAME)\
						bt_node_return_value = CALL_REMOTE(NAME,boost::ref(call_ctx_##NAME), boost::ref(events_##NAME))(); \
					__BT_END_TASK(NAME)
#define BT_CALL_FSM(NAME)\
					__BT_NODE_BGN(BT_TASK, NAME, "")\
						call_ctx.pop();\
						bt_node_return_value = Fsm##NAME(&call_ctx, &events);\
					__BT_END_TASK(NAME)

#define BT_RAISE(EVENT) \
			DMDEBUG( cout<<" RAISE("<<node_name<<":"<<decision_making::Event(#EVENT, call_ctx)<<") "; ) \
			events.raiseEvent(decision_making::Event(#EVENT, call_ctx));

//Deprecated
#define BT_RISE(EVENT) BT_RAISE(EVENT)

#if USE_SHORT_BT_SYNTAX

#define TASK(NAME,...)\
	TASK_BGN(NAME){\
		__VA_ARGS__ \
	}END_TASK(NAME)

#define PAR(NAME,...)\
	PAR_BGN(NAME){\
		__VA_ARGS__ \
	}END_TASK(NAME)
#define SEQ(NAME,...)\
	SEQ_BGN(NAME){\
		__VA_ARGS__ \
	}END_TASK(NAME)
#define SEL(NAME,...)\
	SEL_BGN(NAME){\
		__VA_ARGS__ \
		END_LIST;\
	}END_TASK(NAME)

#define TREE(NAME,...)\
	BT_BGN(NAME){\
		__VA_ARGS__ \
	}BT_END(NAME)
#define ROOT(NAME,...)\
	ROOT_BT_BGN(NAME){\
		__VA_ARGS__ \
	}BT_END(NAME)

#define NOT(...) do{TASK(DECOR_NOT_,\
					__VA_ARGS__;\
					TaskResult res = LAST_NODE->run();\
					TASK_RESULT( res==E_SUCCESS?E_FAIL:E_SUCCESS );\
				 );}while(0)
#endif


struct BTCaller{
	std::string task_address;
	decision_making::CallContext& call_ctx;
	decision_making::EventQueue& queue;
	BTCaller(std::string task_address, decision_making::CallContext& call_ctx, decision_making::EventQueue& queue):
					task_address(task_address), call_ctx(call_ctx), queue(queue)
	{}
	virtual ~BTCaller(){}
	virtual decision_making::TaskResult _bt_function() =0;
	boost::thread* getThread(){ return new boost::thread( boost::bind( &BTCaller::_bt_function,  this )); }
};

#define __BT_CREATE_BT_CALL_FUNCTION(BTNAME, P_call_ctx, P_events_queu)\
				struct _bt_function_struct##BTNAME: public decision_making::BTCaller{\
					_bt_function_structBT1(std::string task_address, decision_making::CallContext& call_ctx, decision_making::EventQueue& queue):decision_making::BTCaller(task_address,call_ctx,queue){}\
					decision_making::TaskResult _bt_function(){\
						BTContext _tmp_context;\
						BT_ROOT_BGN(bt_from_fsm, __tmp_event_queue()){\
							call_ctx.pop();\
							BT_PAR_BGN(TMP){\
								call_ctx.pop();\
								BT_TASK_BGN(STOP_CONDITION){\
									while(not isTerminated())\
									{\
										decision_making::Event e = events.waitEvent();\
										if(not e){\
											DMDEBUG( cout<<" (STOP_CONDITION:TERMINATED) "; )\
											BT_TASK_RESULT( decision_making::TaskResult::TERMINATED() );\
											break;\
										}\
									}\
								}\
								BT_TASK_END(STOP_CONDITION);\
								__BT_CALL_BT_CALLER(BTNAME);\
							}\
							BT_PAR_END(TMP);\
						}\
						BT_END(bt_from_fsm) bt_from_fsm(_tmp_context, call_ctx, queue);\
						decision_making::TaskResult res = bt_from_fsm.run();\
						DMDEBUG( cout<<"(fsm from function finished)"; )\
						return res;\
					}\
				}  _bt_function_struct_instanceBT1(#BTNAME, P_call_ctx, P_events_queu);

#define __CALL_BT_FUNCTION(NAME, CALLS, EVENTS) _bt_function_struct_instance##BT1.getThread()


}

#endif /* BT_H_ */
