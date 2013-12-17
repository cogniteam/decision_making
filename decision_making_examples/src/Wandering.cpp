/*
 * Filename: Wandering.cpp
 *   Author: Igor Makhtes
 *     Date: Dec 16, 2013
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
#include <random_numbers/random_numbers.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

#include <decision_making/FSM.h>
#include <decision_making/ROSTask.h>
#include <decision_making/DecisionMaking.h>

using namespace std;
using namespace decision_making;

#define foreach BOOST_FOREACH

/*************************************************************************************************
*** Variables
**************************************************************************************************/

random_numbers::RandomNumberGenerator _randomizer;
ros::Publisher _velocityPublisher;

/*************************************************************************************************
*** Final state machine
**************************************************************************************************/

FSM(Wandering)
{
    FSM_STATES
    {
        Drive,
        Turn,
        Pause
    }
    FSM_START(Turn)
    FSM_BGN
    {
        FSM_STATE(Turn)
        {
            FSM_CALL_TASK(Turn)

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT(/TURN_TIMEOUT, FSM_NEXT(Drive))
                FSM_ON_EVENT(/PAUSE, FSM_NEXT(Pause))
            }
        }
        FSM_STATE(Drive)
        {
            FSM_CALL_TASK(Drive)

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT(/DRIVE_TIMEOUT, FSM_NEXT(Turn))
                FSM_ON_EVENT(/OBSTACLE, FSM_NEXT(Turn))
                FSM_ON_EVENT(/PAUSE, FSM_NEXT(Pause))
            }
        }
        FSM_STATE(Pause)
        {
            FSM_CALL_TASK(Pause)

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT(/RESUME, FSM_NEXT(Turn))
            }
        }
    }
    FSM_END
}

/*************************************************************************************************
*** ROS Subscriptions
**************************************************************************************************/

void onLaserScanMessage(const sensor_msgs::LaserScan::Ptr laserScanMessage, RosEventQueue* eventQueue) {
    double frontRange = laserScanMessage->ranges[laserScanMessage->ranges.size() / 2];

    if (frontRange < 0.5) {
        eventQueue->riseEvent("/OBSTACLE");
    }
}


/*************************************************************************************************
*** Task implementations
**************************************************************************************************/

decision_making::TaskResult driveTask(string name, const FSMCallContext& context, EventQueue& eventQueue) {
    ROS_INFO("Driving...");

    double timeToDriveMs = _randomizer.uniformInteger(4000, 8000);

    geometry_msgs::Twist forwardMessage;
    forwardMessage.linear.x = 2;
    _velocityPublisher.publish(forwardMessage);

    /**
     * Preemptive wait
     */
    for (int i = 0; i < 100; ++i) {
        if (eventQueue.isTerminated()) {
            ROS_INFO("Obstacle!");
            return TaskResult::TERMINATED();
        }

        boost::this_thread::sleep(boost::posix_time::milliseconds(timeToDriveMs / 100.0));
    }

    eventQueue.riseEvent("/DRIVE_TIMEOUT");
    return TaskResult::SUCCESS();
}

decision_making::TaskResult turnTask(string name, const FSMCallContext& context, EventQueue& eventQueue) {
    ROS_INFO("Turning...");

    bool turnRight = _randomizer.uniformInteger(0, 1);
    int timeToTurnMs = _randomizer.uniformInteger(2000, 4000);

    geometry_msgs::Twist turnMessage;
    turnMessage.angular.z = 2 * (turnRight ? 1 : -1);
    _velocityPublisher.publish(turnMessage);

    boost::this_thread::sleep(boost::posix_time::milliseconds(timeToTurnMs));

    eventQueue.riseEvent("/TURN_TIMEOUT");
    return decision_making::TaskResult::SUCCESS();
}

decision_making::TaskResult pauseTask(string name, const FSMCallContext& context, EventQueue& eventQueue) {
    ROS_INFO("Pausing...");

    geometry_msgs::Twist stopMessage;
    _velocityPublisher.publish(stopMessage);

    return decision_making::TaskResult::SUCCESS();
}

/*************************************************************************************************
*** The Main
**************************************************************************************************/

int main(int argc, char **argv) {
    /**
     * Initialization
     */
    ros::init(argc, argv, "wandering");
    ros_decision_making_init(argc, argv);
    ros::NodeHandle nodeHandle;
    RosEventQueue eventQueue;

    /**
     * Tasks registration
     */
    LocalTasks::registrate("Drive", driveTask);
    LocalTasks::registrate("Turn", turnTask);
    LocalTasks::registrate("Pause", pauseTask);

    /**
     * Subscription for the laser topic and velocity publisher creation
     */

    ros::Subscriber laserSubscriber = nodeHandle.subscribe<void>("/pioneer_1/scan_raw", 1,
                boost::function<void(const sensor_msgs::LaserScan::Ptr)>(boost::bind(onLaserScanMessage, _1, &eventQueue)));

    _velocityPublisher = nodeHandle.advertise<geometry_msgs::Twist>("/pioneer_1/cmd_vel", 100, false);

    /**
     * ROS Spinner for topic subscriptions
     */
    ros::AsyncSpinner spinner(1);
    spinner.start();

    /**
     * Execution of the FSM
     */
    ROS_INFO("Starting wandering machine...");
    FsmWandering(NULL, &eventQueue);

    /**
     * Cleanup
     */
	return 0;
}
