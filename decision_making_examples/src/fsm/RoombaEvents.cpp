
#include <iostream>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Range.h>
#include <random_numbers/random_numbers.h>

using namespace std;

int main(int argc, char** argv){

	ros::init(argc, argv, "fsm_roomba_events");

	ros::NodeHandle node;
	random_numbers::RandomNumberGenerator randomizer;

	ros::Publisher leftBumperPub = node.advertise<std_msgs::Bool>("/roomba/left_bumper", 1, false);
	ros::Publisher rightBumperPub = node.advertise<std_msgs::Bool>("/roomba/right_bumper", 1, false);
	ros::Publisher wallSensorPub = node.advertise<sensor_msgs::Range>("/roomba/wall_sensor", 1, false);

	ROS_INFO("Starting roomba event publisher...");

	while (ros::ok()) {
	    std_msgs::Bool leftBumper, rightBumper;
	    sensor_msgs::Range wallSensor;

	    wallSensor.range = randomizer.uniformReal(0.2, 3);
	    leftBumper.data = randomizer.uniformInteger(1, 5) == 1;
	    rightBumper.data = randomizer.uniformInteger(1, 5) == 1;

	    leftBumperPub.publish(leftBumper);
	    rightBumperPub.publish(rightBumper);
	    wallSensorPub.publish(wallSensor);

	    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
	}

	return 0;
}


