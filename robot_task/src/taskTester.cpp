#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <robot_task/RobotTask.h>
#include <robot_task/RobotTaskAction.h>
using namespace std;
using namespace robot_task;

typedef actionlib::SimpleActionClient<RobotTaskAction> Client;


std::string find(std::vector<string>& s, std::string key){
	for(size_t i=0;i<s.size();i++){
		if(s[i].find(key)==0) return s[i];
	}
	return "";
}
std::string value(std::vector<string>& s, std::string key){
	for(size_t i=0;i<s.size();i++){
		if(s[i].find(key)==0) return s[i].substr(s[i].find('=')+1);
	}
	return "";
}
template<class A> A cast(std::string name){
	std::stringstream n;n<<name;
	A a; n>>a;
	return a;
}


int main (int argc, char **argv)
{
	if(argc<0) return 1;
	std::string tname (argv[1]);
	
	if(tname=="-h" || tname=="--help"){
		cout<<"Robot task tester"<<endl;
		cout<<"   calls robot task with special parameters"<<endl;
		cout<<"Syntax: task_tester [TASK_NAME] [ARG1=VALUE] [ARG2=VALUE] ..."<<endl;
		cout<<"Args:"<<endl;
		cout<<"    time=INTEGER   : time (in sec) for waiting answer from task before abort it. if -1, wait forevere"<<endl;
		cout<<"    arg=ARGS_LIST  : arguments for task call. ARGS_LIST example: x=10,y=12,z=11"<<endl;
		return 0;
	}

	ROS_INFO("Test caller for task [%s] is started.", tname.c_str());
	stringstream sname ; sname << "task_tester"<<"_"<<tname<<"_"<<::time(NULL);
	ros::init(argc, argv, sname.str().c_str());

	// create the action client
	// true causes the client to spin its own thread
	Client ac(tname.c_str(), true);
	
	std::vector<string> params;
	if(argc>2){
		for(int i=2; i<argc; i++){
			params.push_back(argv[i]);
		}
	}
	

	ROS_INFO("Waiting for action server to start task %s.", tname.c_str());
	// wait for the action server to start
	//ac.waitForServer(); //will wait for infinite time
	
	bool serverFound = false;
	while( !serverFound && ros::ok() ){
		ac.waitForServer(ros::Duration(1.0));
		if(ac.isServerConnected()){
			ROS_INFO("... Task is found.");
			serverFound = true;
		}else{
			ROS_INFO("... Task not found, retry...");
		}
	}

	// send a goal to the action
	RobotTaskGoal goal;
		goal.name = tname;
		stringstream suid; suid<<"task_id_tested_"<<::time(NULL);
		goal.uid = suid.str();
		goal.parameters="";
		if(find(params,"arg=").size()!=0)
			goal.parameters = value(params, "arg=");
		else if(find(params,"args=").size()!=0)
			goal.parameters = value(params, "args=");
			
	ROS_INFO("Sending goal.");
	ROS_INFO("   name=%s", goal.name.c_str());
	ROS_INFO("   uid=%s", goal.uid.c_str());
	ROS_INFO("   parameters=%s", goal.parameters.c_str());
	
	ac.sendGoal(goal);
	ROS_INFO("... Goal sent.");

	
	ROS_INFO("Wait for result.");
	
	int time = -1;
	if( find(params, "time=").size()>0 ){
		time = cast<int>(value(params, "time="));
	}
	bool finished = false;
	while(time!=0 && ros::ok()){
		time -- ;
		
		ROS_INFO("... Wait for 1 sec");
		//wait for the action to return
		bool finished_before_timeout = ac.waitForResult(ros::Duration(1.0));

		if (finished_before_timeout)
		{
			actionlib::SimpleClientGoalState state = ac.getState();
			ROS_INFO("... Task finished: %s",state.toString().c_str());
			finished = true;
			if(
				state==actionlib::SimpleClientGoalState::SUCCEEDED ||
				state==actionlib::SimpleClientGoalState::PREEMPTED ||
				state==actionlib::SimpleClientGoalState::ABORTED
			){
				ROS_INFO("... ... Task result is : ");
				ROS_INFO("... ... ... result=%i", ac.getResult()->success);
				ROS_INFO("... ... ... desc=%s", ac.getResult()->description.c_str());
				ROS_INFO("... ... ... plan=%s", ac.getResult()->plan.c_str());
			}
			break;
		}
		else
			ROS_INFO("... ... Task did not finish before the time out. retry...");
	}

	if(!finished && ros::ok()){
		ROS_INFO("Abort task.");
		ac.cancelGoal();
	}
	
	ROS_INFO("Exit from task tester.");
	return 0;
}
