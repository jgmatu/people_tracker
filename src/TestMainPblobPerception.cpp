#include <stdio.h>

#include <ros/ros.h>
#include <ros/console.h>

#include "people_tracker/PlobPerception.h"

int
main(int argc, char** argv)
{
	ros::init(argc , argv , "testperception");
	PlobPerception plob_perception;
	ros::spin();
	return 0;
}
