/*
 * Filename: TaoExample.cpp
 *   Author: Igor Makhtes
 *     Date: Dec 26, 2013
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2013 Cogniteam Ltd.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <iostream>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/foreach.hpp>
#include <ros/ros.h>

#include <decision_making/TAO.h>
#include <decision_making/TAOStdProtocols.h>
#include <decision_making/ROSTask.h>
#include <decision_making/DecisionMaking.h>

using namespace std;
using namespace decision_making;

#define foreach BOOST_FOREACH

TAO_HEADER(Tao2)
TAO_HEADER(Tao3)

#define TAO_STOP_CONDITION(X) TAO_STOP_CONDITION_AND_PRINT_EVENTS(X)

TAO(Tao1)
{
	cout<<"[STR: Tao1]"<<endl;
	struct T{~T(){ cout<<"[END: Tao1]"<<endl; }} t;
    TAO_PLANS{
        Plan1,
        Plan2,
        Plan3
    }
    TAO_START_PLAN(Plan1);
    TAO_BGN{
        TAO_PLAN( Plan1 ){
            TAO_START_CONDITION(true);
            TAO_ALLOCATE(AllocFirstReady){
                TAO_SUBPLAN(Tao2);
                TAO_SUBPLAN(Tao3);
            }
            TAO_CALL_TASK(C);
            TAO_CLEANUP_BGN
            {
                TAO_CALL_TASK(testTask);
            }
            TAO_CLEANUP_END
            TAO_STOP_CONDITION(event == TAO_EVENT(/FAIL))
            TAO_NEXT(NextFirstReady){
                TAO_NEXT_PLAN(Plan3);
                TAO_NEXT_PLAN(Plan2);
            }
        }
        TAO_PLAN( Plan2 ){
            TAO_START_CONDITION(true);
            TAO_CALL_TASK(testTask);
            TAO_ALLOCATE(AllocFirstReady){
                TAO_SUBPLAN(Tao2);
                TAO_SUBPLAN(Tao3);
            }
            TAO_STOP_CONDITION(true);
            TAO_NEXT_EMPTY
        }
        TAO_PLAN( Plan3 ){
            TAO_START_CONDITION(true);
            //TAO_RESULT(SUCCESS, TaskResult::SUCCESS());
            TAO_ALLOCATE_EMPTY
            TAO_STOP_CONDITION(true);
            TAO_NEXT(NextFirstReady){
                TAO_NEXT_PLAN(Plan1);
                TAO_NEXT_PLAN(Plan2);
            }
        }
    }
    TAO_END
}

TAO(Tao2)
{
	cout<<"[STR: Tao2]"<<endl;
	struct T{~T(){ cout<<"[END: Tao2]"<<endl; }} t;
    TAO_PLANS{
        Plan1,
        Plan2
    }
    TAO_START_PLAN(Plan2);
    TAO_BGN
    {
        TAO_PLAN( Plan1 ){
            TAO_START_CONDITION(true);
            TAO_CALL_TASK(testTask);
            TAO_ALLOCATE_EMPTY
            TAO_CLEANUP_BGN
            {
                TAO_CALL_TASK(RRR);
            }
            TAO_CLEANUP_END
            TAO_STOP_CONDITION(event == TAO_EVENT(/FAIL));
            TAO_NEXT(NextFirstReady){
                TAO_NEXT_PLAN(Plan2);
            }
        }
        TAO_PLAN( Plan2 ){
        	cout<<"[start Tao2/Plan2 "<< (justCondition?"condition check":"") <<"]"<<endl;
            TAO_START_CONDITION(true);
            TAO_CALL_TASK(testTask);
            TAO_ALLOCATE_EMPTY
            TAO_CLEANUP_BGN
			{
				cout<<"[stop Tao2/Plan2]"<<endl;
			}
			TAO_CLEANUP_END
			TAO_STOP_CONDITION(true);
            TAO_NEXT_EMPTY
        }
    }
    TAO_END
}

TAO(Tao3)
{
	cout<<"[STR: Tao3]"<<endl;
	struct T{~T(){ cout<<"[END: Tao3]"<<endl; }} t;
    TAO_PLANS{
        Plan1,
        Plan2
    }
    TAO_START_PLAN(Plan1);
    TAO_BGN{
        TAO_PLAN( Plan1 ){
            TAO_START_CONDITION(true);
            TAO_CALL_TASK(testTask);
            TAO_CLEANUP_BGN
            {
                TAO_CALL_TASK(testTask);
            }
            TAO_CLEANUP_END

            TAO_ALLOCATE(AllocFirstReady){
                TAO_SUBPLAN(Tao2);
            }

            TAO_STOP_CONDITION(event == TAO_EVENT(/FAIL))
            TAO_NEXT(NextFirstReady){
                TAO_NEXT_PLAN(Plan2);
            }
        }
        TAO_PLAN( Plan2 ){
            TAO_START_CONDITION(true);
            TAO_CALL_TASK(testTask);
            TAO_ALLOCATE_EMPTY
            TAO_STOP_CONDITION(true);
            TAO_NEXT_EMPTY
        }
    }
    TAO_END
}

decision_making::TaskResult testTask(string name, const FSMCallContext& context, EventQueue& eventQueue) {
	cout<<"[testTask from "<<context.str()<<"]"<<endl;
    sleep(1);
    return TaskResult::SUCCESS();
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "tao_example");

    ROS_INFO("Starting...");

    ros_decision_making_init(argc, argv);
    ros::NodeHandle nodeHandle;
    RosEventQueue eventQueue;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    LocalTasks::registrate("testTask", testTask);

    eventQueue.async_spin();
    ROS_INFO("Starting tao example...");
    TaoTao1(NULL, &eventQueue);
    eventQueue.close();

    ROS_INFO("Tao stopped");
	return 0;
}
