/*
 * BT_BT1.h
 *
 *  Created on: Nov 26, 2013
 *      Author: dan
 */

#ifndef BT_BT1_H_
#define BT_BT1_H_

#include "custom_decision_making.h"

BT_BGN(BT1){
	BT_PAR_BGN(P1){
		BT_CALL_TASK(T1);
		BT_CALL_TASK(T2);
	}
	BT_PAR_END(P1);
}
BT_END(BT1);


#endif /* BT_BT1_H_ */
