/*
 * TAOStdProtocols.h
 *
 *  Created on: Dec 29, 2013
 *      Author: dan
 */

#ifndef TAOSTDPROTOCOLS_H_
#define TAOSTDPROTOCOLS_H_

#include "TAO.h"

namespace decision_making{


class NextFirstReady:public decision_making::ProtocolNext{
public:
	NextFirstReady(int& res, decision_making::CallContext* call_context, decision_making::EventQueue* events):ProtocolNext(res, call_context, events){}
	bool decide(){
		//cout<<"[decide for next of "<<call_context->str()<<" ]";
		for(size_t i=0;i<options.size();i++){
			if(options[i].isReady){
				//cout<<"[ set next state = "<<options[i].id <<": "<<options[i].name<<"]"<<endl;
				result = options[i].id;
				return true;
			}
		}
		return false;
	}
};

class AllocFirstReady:public decision_making::ProtocolAllocation{
public:
	AllocFirstReady(int& res, decision_making::CallContext* call_context, decision_making::EventQueue* events):ProtocolAllocation(res, call_context, events){}
	bool decide(){
		//cout<<"[decide for next of "<<call_context->str()<<" ]";
		for(size_t i=0;i<options.size();i++){
			if(options[i].isReady){
				//cout<<"[ set allocation state = "<<options[i].id <<": "<<options[i].name<<"]"<<endl;
				result = options[i].id;
				return true;
			}
		}
		return false;
	}
};


}


#endif /* TAOSTDPROTOCOLS_H_ */
