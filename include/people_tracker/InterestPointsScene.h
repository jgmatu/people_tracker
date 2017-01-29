/*
 * Borders.h
 *
 *  Created on: May 22, 2016
 *      Author: javi
 */

#ifndef INCLUDE_PEOPLE_TRACKER_INTERESTPOINTSSCENE_H_
#define INCLUDE_PEOPLE_TRACKER_INTERESTPOINTSSCENE_H_


#include <ros/ros.h>
#include <ros/console.h>


#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>


#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int32MultiArray.h>


#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <visualization_msgs/MarkerArray.h>


class InterestPointsScene {

	public:

		typedef struct
		{
			pcl::PointXYZRGB point;
			int i;
			int j;
		}PointInterest;

		InterestPointsScene();
		virtual ~InterestPointsScene();
		std::list<InterestPointsScene::PointInterest> getBorders(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

		void setInterestPoints (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
		const std::list<InterestPointsScene::PointInterest> getInterestPoints () const { return points_ ;};


	private:

		ros::NodeHandle nh_;
		image_transport::ImageTransport it_;

		ros::Publisher markers_pub;
		ros::Subscriber RGBD_sub_;

		tf::TransformListener tfListener_;

		std::string RGBDTopic_;
		std::string workingFrameId_;

		std::list<PointInterest> points_;

		void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& rgbd);
		void publishSphere (visualization_msgs::MarkerArray& markers);
		void publishTriangle (visualization_msgs::MarkerArray& markers);
		void publishPoints ();
		void getMarkerToLines(visualization_msgs::Marker& marker);
		void sample();
		void getMarkerToPoints(visualization_msgs::Marker& marker);


		void cloudCallBack (const sensor_msgs::PointCloud2::ConstPtr& rgbd);

		float getDistance (	InterestPointsScene::PointInterest pointA , InterestPointsScene::PointInterest pointB );


		static const int SAMPLES = 50;
		static const int STEPSW = 24;
		static const int STEPSH = 16;
		static const int FLOOR = 180;
};

#endif /* INCLUDE_PEOPLE_TRACKER_INTERESTPOINTSSCENE_H_ */
