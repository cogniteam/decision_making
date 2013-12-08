#ifndef __DUMMYTASK__HPP
#define __DUMMYTASK__HPP

#include <actionlib/server/simple_action_server.h>
#include <robot_task/RobotTask.h>
#include <robot_task/RobotTaskAction.h>
using namespace std;
using namespace robot_task;

ostream& operator<<(ostream& o, std::vector<string>& s){
	size_t s_size=s.size();
	if(s_size>0){
		for(size_t i=0;i<s_size-1;i++)
			o<<s[i]<<',';
		return o<<s[s.size()-1];
	}
	return o;
}

class DummyTaskServer: public RobotTask {
protected:

	std::vector<string> params;
	
	int time;
	int retValue;
	string outputstr;
	
public:
    DummyTaskServer(std::string name, std::vector<string> par):
        RobotTask(name), params(par)
    {
		std::cout<<"params: "<<params<<std::endl;
		
		time = -1;
		if( find(params, "time=").size()>0 ){
			time = cast<int>(value(params,"time="));
		}
		
		retValue = 0;
		if( find(params, "return=").size()>0 ){
			retValue = cast<int>(value(params,"return="));
		}
		
		outputstr="process...";
		if( find(params, "print=").size()>0 ){
			outputstr = value(params,"print=");
		}
    }

    bool exists(Arguments& args, std::string name){
		return args.find(name) != args.end();
	}
	
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
 
	template<class A> A cast(Arguments& args, std::string name){
		std::stringstream n;n<<args[name];
		A a; n>>a;
		return a;
	}
    
    TaskResult task(const string& name, const string& uid, Arguments& args) {
		int time = this->time;
		
		while(time != 0) {
			time -- ;
			
			if(isPreempt()){
				
				return TaskResult::PREEMPTED();
			}
			
			if(outputstr!="no_print"){
				ROS_INFO("%s: %s", _name.c_str(), outputstr.c_str());
			}
			sleep(1);
		}
                
        
        //return TaskResult::Preempted();
        if( retValue > 0 ){
			return TaskResult(retValue, "ERROR");
		}
        return TaskResult(SUCCESS, "OK");
    }

};
#endif


#include "ros/ros.h"

int main(int argc, char **argv)
{
	if(argc<0) return 1;
	std::string tname (argv[1]);
	
	if(tname=="-h" || tname=="--help"){
		cout<<"Dummy Robot Task for testing"<<endl;
		cout<<"   run empty robot task with any name"<<endl;
		cout<<"Syntax: dummy [TASK_NAME] [ARG1=VALUE] [ARG2=VALUE] ..."<<endl;
		cout<<"Args:"<<endl;
		cout<<"    time=INTEGER    : time (in millisec) for task running. if -1, wait forevere"<<endl;
		cout<<"    return=INTEGER  : return value of task: 0 is OK, -1 is PLAN, 0< is a ERROR CODE "<<endl;
		cout<<"    print=STRING    : print progress text. default is 'process...' and 'no_print' suppress printing."<<endl;
		return 0;
	}
	
	stringstream snodename; snodename<<"DummyTaskNode_"<<time(NULL);
	ROS_INFO("Start %s with Task %s", snodename.str().c_str(),argv[1]);
	
	ros::init(argc, argv, snodename.str().c_str());
	ros::NodeHandle node;
	
	ROS_INFO("craete task");
	

	std::vector<string> params;
	if(argc>2){
		for(size_t i=2; i<argc; i++){
			params.push_back(argv[i]);
		}
	}
	DummyTaskServer task(tname, params);

	ros::spin();

	return 0;
}
