/*
 * FSM_TEST1.h
 *
 *  Created on: Nov 26, 2013
 *      Author: dan
 */

#ifndef FSM_TEST1_H_
#define FSM_TEST1_H_


#include "custom_decision_making.h"

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


#endif /* FSM_TEST1_H_ */
