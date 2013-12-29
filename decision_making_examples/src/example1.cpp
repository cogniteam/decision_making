
#include <ros/ros.h>
#include <decision_making/SynchCout.h>
#include <decision_making/BT.h>
#include <decision_making/FSM.h>
#include <decision_making/ROSTask.h>
#include <decision_making/DecisionMaking.h>

using namespace std;
using namespace decision_making;

EventQueue* mainEventQueue;

FSM(Turnstile)
{
	FSM_STATES
	{
		Locked,
		Unlocked
	}
	FSM_START(Locked);
	FSM_BGN
	{
		FSM_STATE(Locked)
		{
			FSM_CALL_TASK(MYTASK);
			FSM_ON_STATE_EXIT_BGN
			{
				FSM_CALL_TASK(ONEXIT);
			}
			FSM_ON_STATE_EXIT_END

			FSM_TRANSITIONS
			{
				FSM_PRINT_EVENT
				FSM_ON_EVENT(/COIN, FSM_NEXT(Unlocked));
				FSM_ON_EVENT(/PUSH, FSM_NEXT(Locked));
			}
		}
		FSM_STATE(Unlocked)
		{
			FSM_TRANSITIONS
			{
				FSM_PRINT_EVENT
				FSM_ON_EVENT(/COIN, FSM_NEXT(Unlocked));
				FSM_ON_EVENT(/PUSH, FSM_NEXT(Locked));
			}
		}
	}
	FSM_END
}

class PP: public CallContextParameters{
public:
	int x;
	virtual std::string str()const{ std::stringstream s; s<<x; return s.str();};
};
void run_fsm(){
	CallContext ct;
	ct.createParameters<PP>();
	ct.parameters<PP>().x =10;
	FsmTurnstile(&ct, mainEventQueue);
}


void EVENTS_GENERATOR(){
	Event spec[]={"COIN","COIN","PUSH","PUSH", "NOTHING"};
	int i=0;
	boost::this_thread::sleep(boost::posix_time::seconds(1));
	while(true and ros::ok()){
		Event t = spec[i];
		if(t == "NOTHING"){ i=1; t=spec[0]; }else i++;
		cout << endl << t<<" -> ";
		mainEventQueue->riseEvent(t);
		boost::this_thread::sleep(boost::posix_time::seconds(1));
	}
	mainEventQueue->close();
}

class MMM{
	int aa;
public:
	MMM():aa(1){}
TaskResult tst_mytask(std::string task_address, const CallContext& call_ctx, EventQueue& queue, int ww){
	cout<<endl<<"[ this my dummy task("<<task_address<<") : x="<<call_ctx.parameters<PP>().x<<", ww="<<ww<<", aa="<<aa<<" ]"<<endl;
	call_ctx.parameters<PP>().x = call_ctx.parameters<PP>().x + 1;
	aa+=1;
	//cout<<"[ this my dummy task : x=? ]";
	queue.riseEvent(Event("success", call_ctx));
	return TaskResult::SUCCESS();
}
TaskResult tst_onexit(std::string task_address, const CallContext& call_ctx, EventQueue& queue, int ww){
	cout<<endl<<"[ on exit task("<<task_address<<") : x="<<call_ctx.parameters<PP>().x<<", ww="<<ww<<", aa="<<aa<<" ]"<<endl;
	call_ctx.parameters<PP>().x = call_ctx.parameters<PP>().x + 1;
	aa+=1;
	//cout<<"[ this my dummy task : x=? ]";
	queue.riseEvent(Event("success", call_ctx));
	return TaskResult::SUCCESS();
}

};

#include <boost/filesystem.hpp>

int main(int a, char** aa){

	ros::init(a, aa, "RosExample");
	ros_decision_making_init(a, aa);
	mainEventQueue = new RosEventQueue();

	boost::thread_group threads;

	MapResultEvent::map("MYTASK", 0, "success");

	MMM mmm;
	LocalTasks::Function f = boost::bind(&MMM::tst_mytask, &mmm, _1, _2, _3, 15);
	LocalTasks::Function f_onexit = boost::bind(&MMM::tst_onexit, &mmm, _1, _2, _3, 15);
	LocalTasks::registrate("MYTASK", f);
	LocalTasks::registrate("ONEXIT", f_onexit);

	threads.add_thread(new boost::thread(boost::bind(&run_fsm)));
	threads.add_thread(new boost::thread(boost::bind(&EVENTS_GENERATOR)));

	ros::spin();
	threads.join_all();

	delete mainEventQueue;
	return 0;
}


