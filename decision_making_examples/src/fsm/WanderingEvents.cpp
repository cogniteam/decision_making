/*
 * Filename: WanderingEvents.cpp
 *   Author: Igor Makhtes
 *     Date: Jan 8, 2014
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2014 Cogniteam Ltd.
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

using namespace std;

#define foreach BOOST_FOREACH

void publishRandomLaserScan(ros::Publisher& laserScanPublisher) {
    static random_numbers::RandomNumberGenerator randomizer;
    sensor_msgs::LaserScan scan;
    scan.angle_min = 0;
    scan.angle_increment = 0.05;
    scan.header.frame_id = "/map";
    scan.header.stamp  = ros::Time::now();
    scan.range_min = 0.001;
    scan.range_max = 30;

    static double first = randomizer.uniformReal(0.1, 1.5);

    for (int i = 0; i < 50; ++i) {
        first += randomizer.gaussian(0, 0.03);
        first = fmin(1.5, fmax(0.1, first));
        scan.ranges.push_back(first);
    }

    laserScanPublisher.publish(scan);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "fsm_wandering_events");
    ros::Publisher laserScanPublisher = ros::NodeHandle().advertise<sensor_msgs::LaserScan>("/fsm_wandering/scan", 1, false);

    ROS_INFO("Starting wandering events publisher...");

    while (ros::ok()) {
        publishRandomLaserScan(laserScanPublisher);
        boost::this_thread::sleep(boost::posix_time::milliseconds(100));
    }
	return 0;
}
