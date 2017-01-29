/*
 * TestBorders.cpp
 *
 *  Created on: May 28, 2016
 *      Author: javi
 */

#include <stdio.h>

#include <ros/ros.h>
#include <ros/console.h>

#include "people_tracker/PlobPerception.h"

int
main(int argc, char** argv)
{
	ros::init(argc , argv , "testborder");
	InterestPointsScene border;
	ROS_WARN("Borders noded Started.\n");
	ros::spin();
	return 0;
}



