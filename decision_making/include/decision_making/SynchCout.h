/*
 * SynchCout.h
 *
 *  Created on: Nov 14, 2013
 *      Author: dan
 */

#ifndef SYNCHCOUT_H_
#define SYNCHCOUT_H_

#ifndef DISABLE_DECISION_MAKING_LOG

#include <boost/thread.hpp>
#include <deque>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>
#include <boost/foreach.hpp>

using namespace std;


struct Log{
	static boost::mutex& mutex(){ static boost::mutex LogMtx; return LogMtx; }
	boost::mutex::scoped_lock locker;
	Log():locker(mutex()){}
	template<class A>
	Log& operator<<(const A& a){ std::cout<<a; std::cout.flush(); return *this; }
	typedef void(*endl_t)();
	Log& operator<<(endl_t a){ std::cout<<std::endl; std::cout.flush(); return *this; }
};
inline void endl(){}

#define cout Log()

#define DMDEBUG(...) //__VA_ARGS__


#endif /*DISABLE_DECISION_MAKING_LOG*/


#endif /* SYNCHCOUT_H_ */
