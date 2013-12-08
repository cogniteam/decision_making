/*
 * FSM_TEST.h
 *
 *  Created on: Nov 26, 2013
 *      Author: dan
 */

#ifndef FSM_TEST_H_
#define FSM_TEST_H_

#include "custom_decision_making.h"
#include "BT_BT1.h"
#include "FSM_TEST1.h"

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



#endif /* FSM_TEST_H_ */
