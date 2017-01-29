/*
 * cobPeopleAction.cpp
 *
 *  Created on: Jun 6, 2016
 *      Author: javi
 */

#include "ros/ros.h"
#include "ros/console.h"

#include <cob_perception_msgs/DetectionArray.h>
#include "geometry_msgs/Twist.h"

class cobPeopleActionNode{

	public:

		cobPeopleActionNode() :
			cobTrackerTopic_("/detection_tracker/face_position_array"),
			actionTrackerTopic_("/mobile_base/commands/velocity")
		{
			subCobTracker_ = nh_.subscribe(cobTrackerTopic_ , 1000, &cobPeopleActionNode::cobCallback , this);
			pubVelTracker_ = nh_.advertise<geometry_msgs::Twist>(actionTrackerTopic_ , 1000);
		}

		void
		cobCallback(const cob_perception_msgs::DetectionArray::ConstPtr& tracker_msg)
		{
			ROS_WARN ("HEAD DETECTED : %d\n" , (int)(tracker_msg->detections.size()) );

			std::vector<cob_perception_msgs::Detection>::const_iterator it;
			for (it = tracker_msg->detections.begin() ; it != tracker_msg->detections.end() ; it++) {
				std::cerr<<"Head"<<std::endl;
				ROS_INFO ("Position of legs in the image : (x: %lf , y: %lf , z: %lf) \n" , it->pose.pose.position.z , -(it->pose.pose.position.x) , - (it->pose.pose.position.y));
				std::cerr<<it->label<<std::endl;
			}
		}

	private:

		ros::NodeHandle nh_;

		ros::Time node_time;

		ros::Subscriber subCobTracker_;
		ros::Publisher pubVelTracker_;

		std::string cobTrackerTopic_;
		std::string actionTrackerTopic_;
};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "cobAction");

	cobPeopleActionNode cobAction;
	ros::spin();

	return 0;

}
