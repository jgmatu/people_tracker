/*
 * actueNode.cpp
 *
 *  Created on: Jun 15, 2016
 *      Author: javi
 */

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
// #include <csuro_tools/PIDController.h>

const int LEFT = 0;
const int RIGTH = 1;
const int FORWARD = 2;
const int STOP  = 3;
const int MOVES = 4;

const float CENTER = 0.3;
const float NEAR = 4.0;
const float INCERT = 6.0;

void
cmdRigth(geometry_msgs::Twist& cmdVel)
{
	cmdVel.linear.x  =  0.00;
	cmdVel.linear.y  =  0.00;
	cmdVel.linear.z  =  0.00;
	cmdVel.angular.x =  0.00;
	cmdVel.angular.y =  0.00;
	cmdVel.angular.z = -0.33;
}

void
cmdLeft(geometry_msgs::Twist& cmdVel)
{
	cmdVel.linear.x  =  0.00;
	cmdVel.linear.y  =  0.00;
	cmdVel.linear.z  =  0.00;
	cmdVel.angular.x =  0.00;
	cmdVel.angular.y =  0.00;
	cmdVel.angular.z =  0.33;
}

void
send_tf(tf::TransformBroadcaster& br , const tf::StampedTransform& tf , float yaw , std::string child , std::string frame)
{
	tf::StampedTransform map2people;
	tf::Quaternion q;

	map2people.child_frame_id_ = child;
	map2people.frame_id_ = frame;
	map2people.stamp_ = ros::Time::now();

	q.setRPY(0 , 0 , yaw);

	map2people.setRotation(q);
	map2people.setOrigin(tf::Vector3(tf.getOrigin().x(),tf.getOrigin().y(),tf.getOrigin().z()));

	try
	{
		ROS_INFO("Debug people_map : (%lf , %lf , %lf) \n" , map2people.getOrigin().x() , map2people.getOrigin().y() , map2people.getOrigin().z());
		br.sendTransform(map2people);
	}
	catch(tf::TransformException &exception)
	{
		ROS_ERROR("%s", exception.what());
		return;
	}
}

void
publish_vel (ros::Publisher& turtle_vel , tf::TransformBroadcaster& br , const tf::TransformListener& listener , const tf::StampedTransform& bf2people)
{
	geometry_msgs::Twist cmdVel;

//	PIDController angular ("angular" , 0.1 ,  3.0 , 0.00 , 0.80);
//	PIDController linear  ("linear"  , 3.5 ,  7.0 , 0.00 , 0.08);

	float angle_bf 	= atan2 (bf2people.getOrigin().y() , bf2people.getOrigin().x());
	float dist_bf   = sqrt  (bf2people.getOrigin().x() * bf2people.getOrigin().x() + bf2people.getOrigin().y() * bf2people.getOrigin().y());


	tf::StampedTransform map2bf;
	try
	{
		listener.waitForTransform ("/map" , "/base_footprint" , ros::Time(0) , ros::Duration(0.2));
		listener.lookupTransform  ("/map" , "/base_footprint" , ros::Time(0) , map2bf);
	}
	catch (tf::TransformException &exception)
	{
		ROS_ERROR("%s", exception.what());
		return;
	}

	tf::Transform frame2obj;

	frame2obj.setOrigin(tf::Vector3(bf2people.getOrigin().x() , bf2people.getOrigin().y()  , 0));
	frame2obj.setRotation(tf::Quaternion(0, 0 , angle_bf , 0));

	tf::Transform map2people;
	tf::StampedTransform map2peopleStmp;

	map2people = map2bf * bf2people;

	map2peopleStmp.setOrigin(map2people.getOrigin());
	map2peopleStmp.setRotation(map2people.getRotation());

	map2peopleStmp.stamp_ =  bf2people.stamp_;
	map2peopleStmp.frame_id_ =  "/map";
	map2peopleStmp.child_frame_id_ = "people_track_map";

	try
	{
		ROS_INFO("Debug people_map : (%lf , %lf , %lf) \n" , map2peopleStmp.getOrigin().x() , map2peopleStmp.getOrigin().y() , map2peopleStmp.getOrigin().z());
		br.sendTransform(map2peopleStmp);
	}
	catch(tf::TransformException &exception)
	{
		ROS_ERROR("%s", exception.what());
		return;
	}

	float angle_map = atan2 (map2peopleStmp.getOrigin().y() , map2peopleStmp.getOrigin().x());
	float dist_map  = sqrt  (map2peopleStmp.getOrigin().x() * map2peopleStmp.getOrigin().x() + map2peopleStmp.getOrigin().y() * map2peopleStmp.getOrigin().y());

 	//	angular.setReference(angle_bf);
//	linear.setReference(dist_bf);

	cmdVel.angular.x = 0.0;
	cmdVel.angular.y = 0.0;
//	cmdVel.angular.z = angular.getOutput();
//	cmdVel.linear.x  = linear.getOutput();
	cmdVel.linear.x  = 0.0;
	cmdVel.linear.y  = 0.0;
	cmdVel.linear.z  = 0.0;

	ROS_INFO("Angle base_footprint -> tracker  , reference : %lf \n" , angle_bf);
	ROS_INFO("Distance /base_footprint -> /people_track  : %lf \n" ,  dist_bf);

	ROS_INFO ("Angle map -> people : %lf \n" , angle_map);
	ROS_INFO ("Distance map -> people : %lf \n" , dist_map);

	if (dist_bf >= INCERT)
		cmdLeft(cmdVel);

//	turtle_vel.publish(cmdVel);
}

int
main(int argc, char* argv[])
{
	ros::init(argc, argv, "actueNode");
	static ros::NodeHandle nh;
	static ros::Publisher turtle_vel = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);

	static tf::TransformListener listener;
	static tf::TransformBroadcaster br;

	ros::Rate loop_rate(10);
	while (ros::ok())
	{
	    tf::StampedTransform bf2people;
//		tf::StampedTransform people2map;

		try
	    {
//	    	listener.lookupTransform  ("/map" , "/people_track" , ros::Time(0) , people2map);
//	    	listener.waitForTransform ("/map" , "/people_track" , ros::Time(0) , ros::Duration(1.0));
	    	listener.lookupTransform  ("/base_footprint" , "/people_track" , ros::Time(0) , bf2people);
	    	listener.waitForTransform ("/base_footprint" , "/people_track" , ros::Time(0) , ros::Duration(1.0));
	    }
	    catch (tf::TransformException &ex)
	    {
	    	ROS_ERROR("%s",ex.what());
	    	ros::Duration(1.0).sleep();
	    	continue;
	    }
	    publish_vel(turtle_vel , br , listener , bf2people);

	    ros::spinOnce();
		loop_rate.sleep();
	}
}
