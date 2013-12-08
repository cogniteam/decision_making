/*
 * EventSystem.h
 *
 *  Created on: Nov 14, 2013
 *      Author: dan
 */

#ifndef EVENTSYSTEM_H_
#define EVENTSYSTEM_H_


#include <boost/thread.hpp>
#include <deque>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>
#include <boost/foreach.hpp>

namespace decision_making{
	using namespace std;

struct FSMCallContext{
	vector<string> stack;
	FSMCallContext(const FSMCallContext& ctx, string name){
		stack = ctx.stack; push(name);
	}
	FSMCallContext(string name){
		push(name);
	}
	FSMCallContext(){
	}
	string str()const{
		stringstream path;
		BOOST_FOREACH(string s, stack){
			path<<"/"<<s;
		}
		return path.str();
	}
	void push(string name){ stack.push_back(name); }
	void pop(){ stack.pop_back(); }
};
struct Event{
	string _name;
	Event(string lname, const FSMCallContext& ctx){
		if(lname.size()==0){ _name=lname; return; }
		if(lname[0]=='/'){ _name = lname; return; }
		_name = ctx.str()+"/"+lname;
	}
	Event(string lname = ""){
		if(lname.size()==0){ _name=lname; return; }
		if(lname[0]=='/'){ _name = lname; return; }
		_name = "/"+lname;
	}
	Event(const char _lname[]){
		string lname(_lname);
		if(lname.size()==0){ _name=lname; return; }
		if(lname[0]=='/'){ _name = lname; return; }
		_name = "/"+lname;
	}
	string name()const{ return _name; }
	string event_name()const{
		size_t i=0,l=i;
		while(i!=string::npos){
			l=i; i=_name.find('/',l+1);
		}
		return _name.substr(l+1,_name.size());
	}
	bool isUndefined()const{ return _name.size()==0; }
	bool isDefined()const{ return not isUndefined(); }
	bool operator==(const Event& e){ return _name==e._name; }
	bool operator!=(const Event& e){ return _name!=e._name; }
	operator bool()const{ return isDefined(); }
};
inline std::ostream& operator<<(std::ostream& o, Event t){
	return o<<"E["<<t.name()<<']';
}

class EventQueue{
private:
	const bool isTransit;
	mutable boost::mutex events_mutex;
	boost::condition_variable on_new_event;
	std::deque<Event> events;
	bool events_system_stop;
	std::deque<EventQueue*> subs;
	EventQueue* parent;
public:
	EventQueue(EventQueue* parent):isTransit(false),events_system_stop(false),parent(parent){
		if(parent)
			parent->subscribe(this);
	}
	EventQueue(EventQueue* parent, bool isTransit):isTransit(isTransit), events_system_stop(false),parent(parent){
		if(parent)
			parent->subscribe(this);
	}
	EventQueue():isTransit(false),events_system_stop(false),parent(NULL){

	}
	virtual ~EventQueue(){
		if(parent)
			parent->remove(this);
		{
			boost::mutex::scoped_lock l(events_mutex);
			events_system_stop = true;
			on_new_event.notify_all();
			BOOST_FOREACH(EventQueue* sub, subs)
				sub->close();
		}
	}

	virtual void riseEvent(const Event& e){
		if(parent) parent->riseEvent(e);
		else addEvent(e);
	}
private:
	void addEvent(Event e){
		boost::mutex::scoped_lock l(events_mutex);
		if(not isTransit)
			events.push_back(e);
		BOOST_FOREACH(EventQueue* sub, subs) sub->addEvent(e);
		on_new_event.notify_one();
	}
public:
	Event waitEvent(){
		boost::mutex::scoped_lock l(events_mutex);
		while(events_system_stop==false && events.empty())	on_new_event.wait(l);
		if(events_system_stop)
			return Event();
		Event e = events.front();
		events.pop_front();
		return e;
	}
	Event tryGetEvent(bool& success){
		boost::mutex::scoped_lock l(events_mutex);
		success = false;
		if(events_system_stop or events.empty())
			return Event();
		Event e = events.front();
		events.pop_front();
		success = true;
		return e;
	}
	void drop_all(){
		boost::mutex::scoped_lock l(events_mutex);
		events.clear();
	}
public:
	void close(){
		boost::mutex::scoped_lock l(events_mutex);
		events_system_stop = true;
		on_new_event.notify_all();
		BOOST_FOREACH(EventQueue* sub, subs)
			sub->close();
	}
public:
	void subscribe(EventQueue* sub){
		boost::mutex::scoped_lock l(events_mutex);
		if(events_system_stop) return;
		subs.push_back(sub);
	}
private:
	void remove(EventQueue* sub){
		boost::mutex::scoped_lock l(events_mutex);
		for(deque<EventQueue*>::iterator i=subs.begin();i!=subs.end();i++){
			if((*i)==sub){
				subs.erase(i);
				break;
			}
		}
	}
public:
	bool isTerminated()const{
		boost::mutex::scoped_lock l(events_mutex);
		return events_system_stop;
	}
	typedef boost::shared_ptr<EventQueue> Ptr;
};


class MapResultEvent{
	typedef std::map<int,string> mapper_item_type;
	typedef std::map<string, mapper_item_type> mapper_type;
public:
	static mapper_type& get(){ static mapper_type mapper; return mapper; }

	static string map(string task, int result){
		std::stringstream res; res<<result;
		mapper_type& mapper = get();
		if(mapper.find(task)==mapper.end()) return res.str();
		mapper_item_type& item = mapper[task];
		if(item.find(result)==item.end()) return res.str();
		return item[result];
	}
	static void map(string task, int result, string event){
		mapper_type& mapper = get();
		mapper[task][result]=event;
	}
};

}

#endif /* EVENTSYSTEM_H_ */
