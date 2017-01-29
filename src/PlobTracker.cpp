/*
 * PlobTracker.cpp
 *
 *  Created on: 02/05/2016
 *      Author: Paco
 */

#include "people_tracker/PlobTracker.h"

PlobTracker::PlobTracker(int id, double x, double y, const HSVHistogram& histogram , float zmax)
: nh_("~"),
  id_(id)
{
	covariancesMatrix_.setIdentity();
	spaceVelVector_.setZero();

//	ROS_INFO("Value of position in constructor tracker (%lf , %lf) \n" , x , y);

	spaceVelVector_(0) = x;
	spaceVelVector_(1) = y;

	histogram_ = histogram;
	t_ = ros::Time::now();
	correct_ts_= ros::Time::now();

	zmax_ = zmax;

	markers_pub = nh_.advertise<visualization_msgs::MarkerArray>( "/visualization_markerTrackers", 10 );
}

PlobTracker::~PlobTracker()
{
	;
}


// Step predict of Kalman filter.
void
PlobTracker::predict()
{
	dt_ = ros::Time::now() - t_;
	t_  = ros::Time::now();

	Eigen::Vector4f spcVelVec_old(spaceVelVector_); // (sx , sy , vx , vy).

	spaceVelVector_(0)  =  spaceVelVector_(0) + spaceVelVector_(2) * dt_.toSec(); 	 //  sx
	spaceVelVector_(1)  =  spaceVelVector_(1) + spaceVelVector_(3) * dt_.toSec(); 	 //  sy
	spaceVelVector_(3)  =  spaceVelVector_(3) * 0.05;						 		 //  vx
	spaceVelVector_(4)  =  spaceVelVector_(4) * 0.05;						 		 //  vy

	Eigen::Matrix4f Q;
	Q.setIdentity();

	Q(0, 0) = ( spaceVelVector_(0) - spcVelVec_old(0) ) * ( spaceVelVector_(0) - spcVelVec_old(0) ) + ( 0.5 * dt_.toSec() );
	Q(1, 1) = ( spaceVelVector_(1) - spcVelVec_old(1) ) * ( spaceVelVector_(1) - spcVelVec_old(1) ) + ( 0.5 * dt_.toSec() );

	covariancesMatrix_ += Q;
}


float
PlobTracker::getSimilarity(const std::vector<Pblob>::const_iterator instant) const
{
//	return 1.0;
//	this->print();

	float psim = instant->getHistogram().similarity(histogram_);

	float dist_x = spaceVelVector_(0) - instant->getX();
	float dist_y = spaceVelVector_(1) - instant->getY();
	float dist = sqrt (dist_x * dist_x + dist_y * dist_y);

//	fprintf (stderr , "PlobTracker : Distance-> : %lf\n" , dist);
//	fprintf (stderr , "PlobTracker : Position tracker x:%lf y:%lf \n" , spaceVelVector_(0) , spaceVelVector_(1));
//	fprintf (stderr , "PlobTracker : Position plob x:%lf y:%lf \n" , instant->getX() , instant->getY());

	float pdist = pow(E, -dist);

//	fprintf(stderr , "PlobTrakcer : Probabilidad por distancia : %lf \n" , pdist);

	float pdist_x = (spaceVelVector_(2) * dt_.toSec() + spaceVelVector_(0)) - instant->getX();
	float pdist_y = (spaceVelVector_(3) * dt_.toSec() + spaceVelVector_(1)) - instant->getY();
	float distanceP = sqrt (pdist_x * pdist_x + pdist_y * pdist_y);
	float pdist_predict = pow (E , -distanceP);
	float diffZ = fabs(instant->getZmax() - zmax_);

//	ROS_ERROR("class Tracker getSimilarity , diffZ : %lf " , diffZ);

//	ROS_ERROR("prediction dist_x : %lf  \n" , pdist_x);
//	ROS_ERROR("prediction dist_y : %lf  \n" , pdist_y);
//	ROS_ERROR("possible distance : %lf \n" , distanceP);
//	ROS_ERROR("probability the plob and tracker are the same by prediction : %lf \n" , pdistance);

//	if (psim > 0.7 && pdist > 0.80 && pdistance > 0.8)
//		psim = sqrt (psim * pdist * pdistance);

	if (psim > 0.80 && (pdist_predict > 0.9 || pdist > 0.9)) psim = 0.99;

//	ROS_ERROR("DEPURATING SIMILARITY BETWEEN TRACKERS : %lf" , psim);

	return psim;
}

float
PlobTracker::getSimTrack(std::list<PlobTracker>::iterator track)
{
	return histogram_.similarity(track->getHistogram());
}

// Step correct of kalman filter.
void
PlobTracker::correct(const Pblob& perception)
{
	// detect nan values because dt is 0.00 non neccesarry correct then...
	if (dt_.toSec() == 0) {
		dt_ = ros::Time::now() - t_;
		if (dt_.toSec() == 0) return;
	}


	correct_ts_= ros::Time::now();

	histogram_ = perception.getHistogram() * dt_.toSec() * KTE + histogram_ * (1.0 - dt_.toSec());

	Eigen::Vector4f y;
	Eigen::Matrix4f K;
	Eigen::Matrix4f R;
	Eigen::Matrix4f I;

	I.setIdentity();

	y(0) =  perception.getX() - spaceVelVector_(0);
	y(1) =  perception.getY() - spaceVelVector_(1);
	y(2) = (perception.getX() - spaceVelVector_(0)) / dt_.toSec() - spaceVelVector_(2);
	y(3) = (perception.getY() - spaceVelVector_(1)) / dt_.toSec() - spaceVelVector_(3);

/*
	printf("dt: %lf\t (%lf, %lf) -> (%lf, %lf)", dt_.toSec(), static_cast<double>(s_(0)), static_cast<double>(s_(1)), perception.getX(), perception.getY());
	printf("vx = %lf\t Zvx = %lf\n", static_cast<double>(s_(2)), static_cast<double>(y(2)));
	printf("vy = %lf\t Zvy = %lf\n", static_cast<double>(s_(3)), static_cast<double>(y(3)));
*/
	R.setIdentity();
	R(0, 0) = (perception.getXmax() - perception.getXmin()) * (perception.getXmax() - perception.getXmin());
	R(1, 1) = (perception.getYmax() - perception.getYmin()) * (perception.getYmax() - perception.getYmin());;
	R(2, 2) = (R(0, 0) / dt_.toSec()) * (R(0, 0) / dt_.toSec());
	R(2, 2) = (R(1, 1) / dt_.toSec()) * (R(1, 1) / dt_.toSec());;

	Eigen::Matrix4f K1;
	K1 = covariancesMatrix_ + R;
	K = covariancesMatrix_ * K1.inverse();

	spaceVelVector_ = spaceVelVector_ + K * y;
	covariancesMatrix_ = (I-K) * covariancesMatrix_;

	spaceVelVector_(2) = std::max(-0.5 , std::min(0.5, static_cast<double>(spaceVelVector_(2))) );
	spaceVelVector_(3) = std::max(-0.5 , std::min(0.5, static_cast<double>(spaceVelVector_(3))) );

	covariancesMatrix_(0, 0) = std::max(0.0001, static_cast<double>(covariancesMatrix_(0, 0)));
	covariancesMatrix_(1, 1) = std::max(0.0001, static_cast<double>(covariancesMatrix_(1, 1)));
	covariancesMatrix_(2, 2) = std::max(0.0001, static_cast<double>(covariancesMatrix_(2, 2)));
	covariancesMatrix_(3, 3) = std::max(0.0001, static_cast<double>(covariancesMatrix_(3, 3)));
}


void
PlobTracker::print() const
{
	std::cout<<"PlobTracker : ============================="<<std::endl;
	std::cout<<"id = "<<id_<<std::endl;
	std::cout<<std::endl;
	std::cout<<"s : "<<std::endl;
	std::cout<<spaceVelVector_<<std::endl;
	std::cout<<std::endl;
	std::cout<<"Covariances : "<<std::endl;
	std::cout<<std::endl;
	std::cout<<covariancesMatrix_<<std::endl;
	std::cout<<std::endl;
//	histogram_.printP();
	std::cout<<"PlobTracker : ============================="<<std::endl;
}


void
PlobTracker::removeMarkers()
{
	if(markers_pub.getNumSubscribers()== 0) return;

	visualization_msgs::MarkerArray markers;
	markers.markers.clear();

	visualization_msgs::Marker marker;
	marker.header.frame_id = "/base_footprint";
	marker.header.stamp = ros::Time();
	marker.ns = "Plobs";
	marker.id = id_;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::DELETE;
	markers.markers.push_back(marker);

	marker.header.frame_id = "/base_footprint";
	marker.header.stamp = ros::Time();
	marker.ns = "Plobs";
	marker.id = id_ + 100;
	marker.type = visualization_msgs::Marker::ARROW;
	marker.action = visualization_msgs::Marker::DELETE;
	markers.markers.push_back(marker);

	markers_pub.publish( markers );
}

void
PlobTracker::circleIncert (visualization_msgs::MarkerArray& markers , int idtracker) const
{

	visualization_msgs::Marker marker;

	marker.header.frame_id = "/base_footprint";
	marker.header.stamp = ros::Time();
	marker.ns = "Plobs";
//	marker.id = id_ * 2;
	marker.id = id_ + 100;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;

	marker.pose.position.x = static_cast<double>(spaceVelVector_(0));
	marker.pose.position.y = static_cast<double>(spaceVelVector_(1));
	marker.pose.position.z = 1;

	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	marker.scale.x = sqrt(static_cast<double>(covariancesMatrix_(0, 0)));
	marker.scale.y = sqrt(static_cast<double>(covariancesMatrix_(1, 1)));
	marker.scale.z = 0.1;

	marker.color.a = 1.0; // Don't forget to set the alpha!

	if (id_ != idtracker) {
		marker.color.r = 0.0;
		marker.color.g = 0.0;
		marker.color.b = 1.0;
	}else {
		marker.color.r = 1.0;
		marker.color.g = 0.0;
		marker.color.b = 0.0;
	}
	markers.markers.push_back(marker);

}


void
PlobTracker::arrowPredict (visualization_msgs::MarkerArray& markers , int idtracker) const
{

	visualization_msgs::Marker marker;

	marker.header.frame_id = "/base_footprint";
	marker.header.stamp = ros::Time();
	marker.ns = "Plobs";
//	marker.id = id_ * 2 + 1;
	marker.id = id_;
	marker.type = visualization_msgs::Marker::ARROW;
	marker.action = visualization_msgs::Marker::ADD;

	geometry_msgs::Point start, end;

	start.x = 0.0;//static_cast<double>(s_(0));
	start.y = 0.0;//static_cast<double>(s_(1));
	start.z = 0.0;//1.0;

	end.x = static_cast<double>(spaceVelVector_(2));//start.x + static_cast<double>(s_(2));
	end.y = static_cast<double>(spaceVelVector_(3));//start.y + static_cast<double>(s_(3));
	end.z = start.z;

	marker.points.clear();
	marker.points.push_back(start);
	marker.points.push_back(end);

	marker.pose.position.x = static_cast<double>(spaceVelVector_(0));
	marker.pose.position.y = static_cast<double>(spaceVelVector_(1));
	marker.pose.position.z = 1;

	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	marker.scale.x = 0.05;
	marker.scale.y = 0.1;
	marker.scale.z = 0.0;

	marker.color.a = 1.0; // Don't forget to set the alpha!

	if (id_ != idtracker) {
		marker.color.r = 0.0;
		marker.color.g = 0.0;
		marker.color.b = 1.0;
	}else {
		marker.color.r = 1.0;
		marker.color.g = 0.0;
		marker.color.b = 0.0;
	}
	markers.markers.push_back(marker);
}


void
PlobTracker::publishMarkers(int idtracker) const
{
	if(markers_pub.getNumSubscribers() == 0) return; // Rviz is the subcriber to ArrayMarkers :).

	visualization_msgs::MarkerArray markers;
	markers.markers.clear();

	circleIncert(markers , idtracker);
	arrowPredict(markers , idtracker);

	markers_pub.publish( markers );
}

