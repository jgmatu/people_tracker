/*
 * Borders.cpp
 *
 *  Created on: May 22, 2016
 *      Author: javi
 */


#include "../include/people_tracker/InterestPointsScene.h"

InterestPointsScene::InterestPointsScene() :
it_(nh_),
points_(),
RGBDTopic_("/camera/depth_registered/points"),
RGBD_sub_(),
workingFrameId_("/base_footprint")
{
	markers_pub = nh_.advertise<visualization_msgs::MarkerArray>( "visualization_markersInterestPointsInTheScene", 10);
//	RGBD_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(RGBDTopic_, 1, &InterestPointsScene::cloudCallBack, this);
}

InterestPointsScene::~InterestPointsScene()
{

//	ros::NodeHandle nh_;
//	image_transport::ImageTransport it_;

//	ros::Publisher markers_pub;
//	ros::Subscriber RGBD_sub_;

//	tf::TransformListener tfListener_;


//	fprintf (stderr , "\nDeleting interesting points inside the scene...\n");

//	delete &RGBDTopic_;
//	delete &workingFrameId_;

	points_.clear();

//	fprintf (stderr , "\nPoints in the scene deleted...\n");
}

void
InterestPointsScene::publishSphere (visualization_msgs::MarkerArray& markers)
{
	visualization_msgs::Marker marker;

	marker.header.frame_id = workingFrameId_;
	marker.header.stamp = ros::Time();
	marker.ns = "Sphere";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 1;
	marker.pose.position.y = 1;
	marker.pose.position.z = 1;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 1;
	marker.scale.y = 1;
	marker.scale.z = 1;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;

	markers.markers.push_back(marker);
}



void
InterestPointsScene::publishTriangle (visualization_msgs::MarkerArray& markers)
{
	visualization_msgs::Marker marker;
	geometry_msgs::Point point;

	marker.header.frame_id = workingFrameId_;
	marker.header.stamp = ros::Time();
	marker.ns = "Sphere";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::LINE_LIST;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.01;
	marker.scale.y = 0;
	marker.scale.z = 0;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;

	point.x = 0;
	point.y = 0;
	point.z = 0;
	marker.points.push_back(point);

	point.x = 1;
	point.y = 1;
	point.z = 1;
	marker.points.push_back(point);

	point.x = 1;
	point.y = 1;
	point.z = 1;
	marker.points.push_back(point);

	point.x = 2;
	point.y = 1;
	point.z = 0;
	marker.points.push_back(point);

	point.x = 2;
	point.y = 1;
	point.z = 0;
	marker.points.push_back(point);

	point.x = 0;
	point.y = 0;
	point.z = 0;
	marker.points.push_back(point);

	markers.markers.push_back(marker);
}


void
InterestPointsScene::getMarkerToPoints(visualization_msgs::Marker& marker)
{
	geometry_msgs::Point point;

	marker.header.frame_id = workingFrameId_;
	marker.header.stamp = ros::Time();
	marker.ns = "Sphere";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::POINTS;
	marker.action = visualization_msgs::Marker::ADD;

	marker.pose.position.x = 0.0;
	marker.pose.position.y = 0.0;
	marker.pose.position.z = 0.0;

	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	marker.scale.x = 0.01;
	marker.scale.y = 0.01;
	marker.scale.z = 0;

	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;

}

void
InterestPointsScene::publishPoints()
{
	visualization_msgs::MarkerArray markers;
	visualization_msgs::Marker marker;


	markers.markers.clear();
	getMarkerToPoints(marker);

	std::list<PointInterest>::iterator it;
	for (it = points_.begin() ; it != points_.end() ; it++) {
		geometry_msgs::Point point;
		point.x = it->point.x;
		point.y = it->point.y;
		point.z = it->point.z;
		marker.points.push_back(point);
	}
	markers.markers.push_back(marker);
	markers_pub.publish( markers );
}


void
InterestPointsScene::sample()
{
	std::list <PointInterest>::iterator it;
	int count = 0;

	for (it = points_.begin() ; it != points_.end() ; it++) {
		if (count%SAMPLES != 0) {
			it = points_.erase(it);
		}
		count++;
	}
}



float
InterestPointsScene::getDistance (	InterestPointsScene::PointInterest pointA , InterestPointsScene::PointInterest pointB )
{
	return sqrt ((pointA.point.x - pointB.point.x) * (pointA.point.x - pointB.point.x) +
			(pointA.point.y - pointB.point.y) * (pointA.point.y - pointB.point.y) +
			(pointA.point.z - pointB.point.z) * (pointA.point.z - pointB.point.z) );

}
/*
std::list<PointsScene::PointBorder>
PointsScene::getBorders(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{

	int numPoints = 0;
	float distance = 0.0;

	PointBorder last;
	PointBorder point;

	last.point.x = 0;
	last.point.y = 0;
	last.point.z = 0;
	last.j = 0;
	last.i = 0;

	point.point.x = 0;
	point.point.y = 0;
	point.point.z = 0;
	point.j = 0;
	point.i = 0;

	for (int i = 0 ; i < cloud->height ; i++) {
		for (int j = 0 ; j < cloud->width ; j++) {

			int posdata = i * cloud->width + j;

			if(!std::isnan(cloud->at(posdata).x)) {

				point.i = i;
				point.j = j;
				point.point = cloud->at(posdata);

				distance = getDistance(point , last);

				if (distance > 0.5 && point.point.x < last.point.x)
					points_.push_back(point);
				else if (distance > 0.5)
					points_.push_back(last);
				last = point;
			}
		}
	}
	sample();
	publishPoints();
	return points_;
}
*/

void
InterestPointsScene::setInterestPoints (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	PointInterest point;

	for (int i = 0 ; i < cloud->height - FLOOR ; i += STEPSH) {
		for (int j = 0 ; j < cloud->width ; j += STEPSW) {

			int posdata = i * cloud->width + j;

			if (!std::isnan(cloud->at(posdata).x)) {
				point.i = i;
				point.j = j;
				point.point = cloud->at(posdata);
				points_.push_back(point);
			}
		}
	}
//	publishPoints();
}


//void
//InterestPointsScene::cloudCallBack (const sensor_msgs::PointCloud2::ConstPtr& rgbd)
//{
//	try {

//		ROS_WARN("POINTS CLOUD CALLED FROM BODERS NODE ;)\n");

//		sensor_msgs::PointCloud2 rgbd_bf;

		/*
		 * If we are not able to pass base foot print image we can't process the point cloud return from handler.
		 * in this code is necessary have base foot print running.
		 */
//		pcl_ros::transformPointCloud(workingFrameId_, *rgbd, rgbd_bf, tfListener_); // This sentence can get the catch...

//		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_orig(new pcl::PointCloud<pcl::PointXYZRGB>);

//		pcl::fromROSMsg(rgbd_bf, *cloud);
//		pcl::fromROSMsg(rgbd_bf, *cloud_orig);

//		if(cloud->empty()) return;

//		setInterestPoints(cloud);
//		publishPoints();

//	} catch(tf::TransformException& ex){

//		ROS_ERROR_STREAM( "Transform error of sensor data: " << ex.what() << ", quitting callback");
//		return;

//	}

//}
