/*
 * Parsers.h
 *
 *  Created on: Nov 27, 2013
 *      Author: dan
 */

#ifndef PARSERS_H_
#define PARSERS_H_


#include "BTConstructor.h"
#include "FSMConstructor.h"

class FSMParser;
class BTParser;

FSMParser* createFSM(std::string filename);
BTParser* createBT(std::string filename);

void del(FSMParser*);
void del(BTParser*);

bt_constructor::BTConstructor& parseBT(BTParser*);
fsm_constructor::FSMConstructor& parseFSM(FSMParser*);


#endif /* PARSERS_H_ */
