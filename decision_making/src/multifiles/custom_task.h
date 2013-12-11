/*
 * custom_task.h
 *
 *  Created on: Nov 26, 2013
 *      Author: dan
 */

#ifndef CUSTOM_TASK_H_
#define CUSTOM_TASK_H_

#include "main_include.h"

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



#endif /* CUSTOM_TASK_H_ */
