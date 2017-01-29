/*
 * PlobTracker.h
 *
 *  Created on: 02/05/2016
 *      Author: paco
 */

#ifndef PLOBTRACKER_H_
#define PLOBTRACKER_H_

#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>
#include "people_tracker/HSVHistogram.h"
#include "people_tracker/Pblob.h"

class PlobTracker {

public:

	PlobTracker(int id_, double x, double y, const HSVHistogram& histogram , float zmax);
	virtual ~PlobTracker();

	void predict();
	void correct(const Pblob& perception);

	ros::Time getLastCorrected() const {return correct_ts_;};
	float getSimilarity(const std::vector<Pblob>::const_iterator plob) const;

	const HSVHistogram& getHistogram() const {return histogram_;};

	void publishMarkers(int idtracker) const;
	void removeMarkers();

	const Eigen::Matrix4f& getCovariancesMatrix() const { return covariancesMatrix_; };
	const Eigen::Vector4f& getSpaceVelVector() const { return spaceVelVector_; };

	int getId() const { return id_; };

	float getX() const
	{
		if (!spaceVelVector_.hasNaN())
			return spaceVelVector_(0);

		ROS_ERROR("NAN VALUES DETECTED RETURNING X POSE TRACKER!!!!\n");
		return 0.0;
	};

	float getY() const
	{
		if (!spaceVelVector_.hasNaN())
			return spaceVelVector_(1);

		ROS_ERROR("NAN VALUES DETECTED RETURNING Y POSE TRACKER!!!!\n");
		return 0.0;
	};

	float getSimTrack(std::list<PlobTracker>::iterator track);


	void print() const;


private:

	void circleIncert (visualization_msgs::MarkerArray& markers , int idtracker) const;
	void arrowPredict (visualization_msgs::MarkerArray& markers , int idtracker) const;
	visualization_msgs::Marker getRectangleTemplate() const;


	//void clearMarkers();
	//void publishPlob(std::vector<Pblob>::const_iterator& plob, float value, int counter);
	int id_;
	HSVHistogram histogram_;

	Eigen::Vector4f spaceVelVector_;    // Vector : (0.0 , 0.0 , 0.0 , 0.0) :). sx, sy , vx , vy. 4x1
	Eigen::Matrix4f covariancesMatrix_; // Matrix : 4x4 Matrix of floats :).

	ros::Time t_;
	ros::Duration dt_;
	ros::Time correct_ts_;

	ros::NodeHandle nh_;
	ros::Publisher markers_pub;

	float zmax_; // Get siilarity by heigh...

	static const float E = 2.718281828;
	static const float KTE = 2.0;

};

#endif /* PLOBTRACKER_H_ */
