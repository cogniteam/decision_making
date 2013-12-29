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
#include "TAOConstructor.h"

class FSMParser;
class BTParser;
class TAOParser;

FSMParser* createFSM(std::string filename);
BTParser* createBT(std::string filename);
TAOParser* createTAO(std::string filename);

void del(FSMParser*);
void del(BTParser*);
void del(TAOParser*);

bt_constructor::BTConstructor& parseBT(BTParser*);
fsm_constructor::FSMConstructor& parseFSM(FSMParser*);
tao_constructor::TAOConstructor& parseTAO(TAOParser*);


#endif /* PARSERS_H_ */
