/*
 * ParserExceptions.h
 *
 *  Created on: Dec 1, 2013
 *      Author: dan
 */

#ifndef PARSEREXCEPTIONS_H_
#define PARSEREXCEPTIONS_H_

#include <sstream>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <errno.h>

class ParserException{
protected:
	ParserException(std::string desc=""):desc(desc){};
	virtual ~ParserException(){};
	std::string desc;
public:
	std::string what()const{ return desc; }
	template <class A>
	ParserException& operator<<(const A& a){
		std::stringstream s;
		s<<a;
		desc += s.str();
		return *this;
	}
};

class PEFileNotFound:public ParserException{
public:
	PEFileNotFound(std::string filename):ParserException("File "+filename+" does not found. ")
	{
		(*this)<<"\n\tIOError: "<<strerror( errno );
	}
};

class PEFileNotCreated:public ParserException{
public:
	PEFileNotCreated(std::string filename):ParserException("File "+filename+" cann't be open. ")
	{
		(*this)<<"\n\tIOError: "<<strerror( errno );
	}
};

#endif /* PARSEREXCEPTIONS_H_ */
