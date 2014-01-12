/*
 * Filename: TaoExample.cpp
 *   Author: Igor Makhtes
 *     Date: Dec 26, 2013
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2013 Cogniteam Ltd.
 *
7 * Permission is hereby granted, free of charge, to any person obtaining a copy
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

TAO_HEADER(Even)
TAO_HEADER(Odd)

//#define TAO_STOP_CONDITION(X) TAO_STOP_CONDITION_AND_PRINT_EVENTS(X)

struct WorldModel: public CallContextParameters{
	int i;
	string str()const{ stringstream s; s<<"i="<<i; return s.str(); }
};
#define WM call_ctx.parameters<WorldModel>()

TAO(Incrementer)
{
    TAO_PLANS{
        Increment,
    }
    TAO_START_PLAN(Increment);
    TAO_BGN{
        TAO_PLAN( Increment ){
            TAO_START_CONDITION( WM.i < 100 );
            TAO_ALLOCATE(AllocFirstReady){
                TAO_SUBPLAN(Even);
                TAO_SUBPLAN(Odd);
            }
            TAO_STOP_CONDITION( false );
            TAO_NEXT(NextFirstReady){
            	TAO_NEXT_PLAN(Increment);
            }
        }
    }
    TAO_END
}

TAO(Even)
{
    TAO_PLANS{
        Even,
    }
    TAO_START_PLAN(Even);
    TAO_BGN
    {
        TAO_PLAN( Even ){
            TAO_START_CONDITION( WM.i%2 == 0 );
            WM.i++;
            cout<<"Even : "<<WM.str()<<endl;
            sleep(1);
            TAO_ALLOCATE_EMPTY
            TAO_STOP_CONDITION(true);
            TAO_NEXT_EMPTY
        }
    }
    TAO_END
}

TAO(Odd)
{
    TAO_PLANS{
        Odd,
    }
    TAO_START_PLAN(Odd);
    TAO_BGN
    {
        TAO_PLAN( Odd ){
            TAO_START_CONDITION( WM.i%2 != 0 );
            WM.i++;
            cout<<"Odd : "<<WM.str()<<endl;
            sleep(1);
            TAO_ALLOCATE_EMPTY
            TAO_STOP_CONDITION(true);
            TAO_NEXT_EMPTY
        }
    }
    TAO_END
}


decision_making::TaskResult dummyTask(string name, const FSMCallContext& context, EventQueue& eventQueue) {
	cout<<"[testTask from "<<context.str()<<"]"<<endl;
    sleep(1);
    return TaskResult::SUCCESS();
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "tao_example_incrementer");

    ROS_INFO("Starting...");

    ros_decision_making_init(argc, argv);
    ros::NodeHandle nodeHandle;
    RosEventQueue eventQueue;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    LocalTasks::registrate("testTask", dummyTask);

    eventQueue.async_spin();
    CallContext call_ctx;
    call_ctx.createParameters(new WorldModel());
    TaoIncrementer(&call_ctx, &eventQueue);
    eventQueue.close();
    ROS_INFO("Finished.");
	return 0;
}
