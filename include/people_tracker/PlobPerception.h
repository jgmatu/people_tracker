/*
 * PlobPerception.h
 *
 *  Created on: 02/05/2016
 *      Author: paco
 */

#ifndef PLOBPERCEPTION_H_
#define PLOBPERCEPTION_H_

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
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>
#include <math.h>

#include "InterestPointsScene.h"
#include "people_tracker/Pblob.h"
#include "people_tracker/Head.h"
#include "people_tracker/Legs.h"

//#define MIN_Z  -0.10		rosbag play youConneNotTouch.bag
//#define MAX_Z	1.90

//#define MIN_Z   0.7
//#define MAX_Z   2.1

#define MIN_Z   0.00	// Real Kobuki on the floor.
#define MAX_Z   2.30


//#define MIN_Z  -0.10  // Real kobuki on the table.
//#define MAX_Z	1.20


// #define DIST_TOLERANCE	0.019

#define DIST_TOLERANCE 0.10

//#define MIN_PEOPLE_SIZE 0.30
//#define MAX_PEOPLE_SIZE 0.71

#define MIN_PEOPLE_SIZE 0.15
#define MAX_PEOPLE_SIZE 0.90

#define MAX_MARKERS	40
#define MIN_PEOPLE_POINTS   7000
#define MAX_PEOPLE_POINTS  55000
#define MIN_DIST_TO_CAMERA 0.10

#define MAX_TIME 5.0
// #define THRESHOLD 0.90   Kobuki Real.
#define THRESHOLD 0.85
#define THRESHOLDLIGTH 0.80
#define MINPROBABILITY 0.0
#define MAXPROBABILITY 1.0

// In the scene.
class PlobPerception {
public:

	PlobPerception();
	virtual ~PlobPerception();

	// We get from camera point clouds the list of blobs that the sensor has
	// catch from the point cloud.
	std::vector<Pblob>& getPerception() { return permanents_; };
	void clear() { permanents_.clear(); };

	int getCountNumsFramesTrack() const { return countNumsFramesTrack; };

	const std::vector<Pblob>& getPermanents() const { return permanents_; };
	void setPermanents(const std::vector<Pblob>& permanents) { permanents_ = permanents; };

private:

	// Obtain plobs from point cloud.
	void setInstantsFromInterestPoints (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud , pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_orig , std::vector < Pblob >& plobs);
	void tracePlobs (std::vector < Pblob > plobs);

	 //Add points to the actual plob getting from cloud...
	void addPoints2Plob(int i, int j, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, float dist , Pblob& plob);


	void checkSimColor (std::vector < Pblob >& instants);

	// Reduce the number of blobs inside the scene.
	void reducePlobs(std::vector< Pblob >& plobs);

	// The blobs must be the size similar to a human blob.
	bool isFalsePerson(float sx , float sy , float sz);
	void checkPlobsSize(std::vector< Pblob >& instants);

	// Compare the blobs in the scene with my permanents blobs.
	void pairPlobs(const std::vector< Pblob >& instant, std::vector< Pblob >& permanent);

	// Publish a marker elipse of incertidumbre that represent the blob
	// in the scene.
	void publish_plobs(const std::vector< Pblob >& plobs);

	// Erase plobs that's not have singularities of a person.
	bool isSingularsPerson (const Pblob& instant);
	void singularities (std::vector <Pblob> & instants);


	void publish_points_dep (const std::list<pcl::PointXYZRGB>& points);

	// Handler of PointCloud in the scene catch with the camera.
	void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& rgbd);

	float getDistance (pcl::PointXYZRGB point);
	float getDistance (pcl::PointXYZRGB pointA , pcl::PointXYZRGB pointB);
	float getDistance (geometry_msgs::Point pointA , geometry_msgs::Point pointB);
	float getDistance (InterestPointsScene::PointInterest point);

	void setTableSims ( Eigen::MatrixXf& simTable , const std::vector < Pblob >& instant, const std::vector < Pblob >& permanent);
	void printTableSims (const Eigen::MatrixXf& simTable , int sizeInst , int sizePerm);

	void update(std::vector < Pblob >& permanent , int id , const Pblob& instant);
	void add(std::vector < Pblob >& permanent , const Pblob& instant);
	void erase(std::vector < Pblob >& permanent);

	static const float KTE_Z = 1.2;

	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;

	std::string workingFrameId_;
	std::string RGBDTopic_;
	std::string DEPTopic_;


	// Attributes that defines a people blob in the scene.
	double min_z_;
	double max_z_;
	double dist_tolerance_;
	double max_people_size_;
	double min_people_size_;
	int min_people_points_;
	int max_people_points_;

	int countNumsFramesTrack;

	ros::Subscriber RGBD_sub_;

	ros::Publisher markers_pub_container;
	ros::Publisher markers_pub_depuration;
	ros::Publisher pclPub;

	tf::TransformListener tfListener_;

	// Save in a list permanents plobs.
	std::vector < Pblob > permanents_;
};

#endif /* PLOBPERCEPTION_H_ */
