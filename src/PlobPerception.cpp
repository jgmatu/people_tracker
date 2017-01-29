/*
 * PlobPerception.cpp
 *
 *  Created on: 02/05/2016
 *      Author: Paco
 */


#include "people_tracker/PlobPerception.h"

PlobPerception::PlobPerception() :
it_(nh_),
workingFrameId_("/base_footprint"),
RGBDTopic_("/camera/depth_registered/points"),
DEPTopic_("/PointDep"),
RGBD_sub_(),
countNumsFramesTrack(0)
{
	ros::NodeHandle private_nh("~");

	private_nh.param("min_z", min_z_, MIN_Z);
	private_nh.param("max_z", max_z_, MAX_Z);
	private_nh.param("dist_tolerance", dist_tolerance_ , DIST_TOLERANCE);
	private_nh.param("max_people_size", max_people_size_ , MAX_PEOPLE_SIZE);
	private_nh.param("min_people_size", min_people_size_ , MIN_PEOPLE_SIZE);
	private_nh.param("min_people_points", min_people_points_ , MIN_PEOPLE_POINTS);
	private_nh.param("max_people_points", max_people_points_ , MAX_PEOPLE_POINTS);

	pclPub = nh_.advertise<sensor_msgs::PointCloud2>(DEPTopic_ , 1 , false);
	RGBD_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(RGBDTopic_, 1, &PlobPerception::cloudCallback, this);
	markers_pub_container = nh_.advertise<visualization_msgs::MarkerArray>( "visualization_markerPlobs", 10);
	markers_pub_depuration = nh_.advertise<visualization_msgs::MarkerArray>( "visualization_markerDepSiluete", 10);
}

PlobPerception::~PlobPerception()
{
	;
}

float
PlobPerception::getDistance (geometry_msgs::Point pointA , geometry_msgs::Point pointB)
{
	return sqrt ( (pointA.x - pointB.x) * (pointA.x - pointB.x) + (pointA.y - pointB.y) * (pointA.y - pointB.y) + (pointA.z - pointB.z) * (pointA.z - pointB.z) );
}

float
PlobPerception::getDistance (pcl::PointXYZRGB pointA , pcl::PointXYZRGB pointB)
{
	return sqrt ( (pointA.x - pointB.x) * (pointA.x - pointB.x) + (pointA.y - pointB.y) * (pointA.y - pointB.y) + (pointA.z - pointB.z) * (pointA.z - pointB.z) );
}


float
PlobPerception::getDistance (pcl::PointXYZRGB point)
{
	return sqrt ( (point.x) * (point.x) + (point.y) * (point.y) + (point.z) * (point.z) );
}


float
PlobPerception::getDistance (InterestPointsScene::PointInterest point)
{
	return sqrt ( (point.point.x) * (point.point.x) + (point.point.y) * (point.point.y) + (point.point.z) * (point.point.z) );
}

void
PlobPerception::publish_points_dep (const std::list<pcl::PointXYZRGB>& points)
{
	std::list < pcl::PointXYZRGB > :: const_iterator it;
	visualization_msgs::MarkerArray markers;
	visualization_msgs::Marker marker;

	marker.header.frame_id = workingFrameId_;
	marker.header.stamp = ros::Time();
	marker.ns = "Plobs";
	marker.id = 1;
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
	marker.scale.z = 0.0;

	marker.color.a = 1.0; // Don't forget to set the alpha! why? ROS documentation about visual markers visibility of object... :).

	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;

	geometry_msgs::Point previous;

	previous.x = points.front().x;
	previous.y = points.front().y;
	previous.z = points.front().z;

	for (it = points.begin() ; it != points.end() ; it++){

		geometry_msgs::Point point;
		point.x = it->x;
		point.y = it->y;
		point.z = it->z;

		float distance = getDistance(point , previous);

		if (distance < 0.1) {
			marker.points.push_back(point);
			marker.points.push_back(previous);
		}
		previous = point;
	}

	markers.markers.push_back(marker);
	markers_pub_depuration.publish(markers);
}

void
PlobPerception::publish_plobs(const std::vector< Pblob > & instants)
{

	if(markers_pub_depuration.getNumSubscribers() == 0) return;

	visualization_msgs::MarkerArray markers;
	std::vector < Pblob > :: const_iterator it;
	int counter = 0;

	/* Site the markers in the point cloud with the perceptions of plobs. */

//	fprintf (stderr , "Pblobs detected : %d\n" , static_cast<int>(plobs.size()));
	for(it = instants.begin(); it != instants.end(); ++it)
	{
		Head head(*it);
		head.isHead();
//		if(( (ros::Time::now() - plobs[counter].updated_ts_).toSec() < 1.0) && ( (ros::Time::now() - plobs[counter].detected_ts_).toSec() > 1.0) )
//		{
//			visualization_msgs::Marker marker;
//			marker.header.frame_id = workingFrameId_;
//			marker.header.stamp = ros::Time();
//			marker.ns = "Plobs";
//			marker.id = counter;
//			marker.type = visualization_msgs::Marker::CUBE;
//			marker.action = visualization_msgs::Marker::ADD;
//			marker.pose.position.x = (plobs[counter].getXmin() + plobs[counter].getXmax()) / 2.0;
//			marker.pose.position.y = (plobs[counter].getYmin() + plobs[counter].getYmax()) / 2.0;
//			marker.pose.position.z = (plobs[counter].getZmin() + plobs[counter].getZmax()) / 2.0;
//			marker.pose.orientation.x = 0.0;
//			marker.pose.orientation.y = 0.0;
//			marker.pose.orientation.z = 0.0;
//			marker.pose.orientation.w = 1.0;
//			marker.scale.x = plobs[counter].getXmax() - plobs[counter].getXmin();
//			marker.scale.y = plobs[counter].getYmax() - plobs[counter].getYmin();
//			marker.scale.z = plobs[counter].getZmax() - plobs[counter].getZmin();
//			marker.color.a = 0.5; // Don't forget to set the alpha! why? ROS documentation about visual markers visibility of object... :).
//			marker.color.r = 0.0;
//			marker.color.g = 1.0;
//			marker.color.b = 0.0;

//			markers.markers.push_back(marker);
//			counter++;

//			if(counter >= MAX_MARKERS)
//			{
//				ROS_WARN("Max number of markers to display %d is exceeded", MAX_MARKERS);
//				break;
//			}
//		}
//		publish_points_dep(head.getHead());// Depurate siluete discoment this line only.
		publish_points_dep(it->getSiluete());// Depurate siluete discoment this line only.
//		publish_points_dep(it->getHistogram().getDep());
//		it->printPoints();
//		markers.markers.push_back(*(it->getRectangleTemplate()));
//		markers.markers.push_back(marker);
	}

	/*
	 * Delete markers not used...
	 */

//	for(int i = (counter - 1) ; i < MAX_MARKERS; i++)
//	{
//		visualization_msgs::Marker marker;
//		marker.header.frame_id = workingFrameId_;
//		marker.header.stamp = ros::Time();
//		marker.ns = "Plobs";
//		marker.id = i;
//		marker.action = visualization_msgs::Marker::DELETE;
//		markers.markers.push_back(marker);
//	}
	markers_pub_container.publish( markers );
}


/**
 * \brief Recursive function to add points to a blob.
 * \param[in]	i, j	Coordinates if the point in image coordinates.
 * \param[in]	cloud	Pointer to the referenced (copied) cloud.
 * \param[in]	dist	Distance of the previous point.
 * \param[out]	plob	Plob to store the the points corresponding to the current plob.
 *
 * It recursively iterates over the cloud using image coordinates.
 * The condition for adding a point to a blob is similar distance to previous point.
 */
void
PlobPerception::addPoints2Plob(int i, int j, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
		float dist, Pblob& plob)
{

	//Test if it is a valid image coordinate.
	if( (i < 0) || (i >= cloud->height) || (j < 0) || (j >= cloud->width)) return;

//	int posdata = j * cloud->width + i;
	int posdata = i * cloud->width + j;

	if(posdata >= cloud->size()) return;

	if (plob.getNPoints() > MAX_PEOPLE_POINTS) return;

	//Test for NaN.
	if(std::isnan(cloud->at(posdata).x)) return;

	//Distance Stop condition.
	float current_dist = getDistance(cloud->at(posdata));
	if(current_dist < 0.1) return;
	if(fabs(current_dist - dist) > dist_tolerance_) return;

	//Z stop condition.
	if((cloud->at(posdata).z > max_z_) || (cloud->at(posdata).z < min_z_)) return;

	//If test passed, include in current plob.
	plob.add( posdata );

	//Mark point as visited.
	cloud->at(posdata).x = 0.0;
	cloud->at(posdata).y = 0.0;
	cloud->at(posdata).z = 0.0;

	//Recursive calls.
	addPoints2Plob(i + 1,  j, cloud, current_dist, plob);
	addPoints2Plob(i - 1,  j, cloud, current_dist, plob);
	addPoints2Plob(i , j + 1, cloud, current_dist, plob);
	addPoints2Plob(i , j - 1, cloud, current_dist, plob);
}



void
PlobPerception::tracePlobs (std::vector < Pblob > plobs)
{
	// Print Results of getPlobs from cloud.
	fprintf(stderr , "PlobPerception  : =========================\n");
	fprintf(stderr , "PlobPerception  : Candidates to plob... : %d\n" , static_cast<int> (plobs.size()) );
	for (int i = 0 ; i < plobs.size() ; i++) {
		fprintf(stderr , "PlobPerception  : Plob%d Position : : (%lf , %lf , %lf) \n" , i , plobs.at(i).getX() , plobs.at(i).getY() , plobs.at(i).getZ());
		fprintf(stderr , "PlobPerception  : Dimesions Pblob%d : (sx : %lf , sy : %lf , sz : %lf) \n" , i , plobs.at(i).getXmax() - plobs.at(i).getXmin() ,
																						 plobs.at(i).getYmax() - plobs.at(i).getYmin() ,
																						 plobs.at(i).getZmax() - plobs.at(i).getZmin());
		fprintf(stderr , "PlobPerception  : Points : %d \n" , plobs[i].getNPoints());
//		plobs.at(i).printSlices();
		std::cerr<<std::endl;
	}
	fprintf(stderr , "=========================\n");
}


void
PlobPerception::setInstantsFromInterestPoints (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud , pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_orig , std::vector < Pblob > & instants)
{
	std::list <InterestPointsScene::PointInterest> interestPoints;

	// This save the points detected in the scene uniforms to get plobs later about this points.
	InterestPointsScene interestPointsScene;

	interestPointsScene.setInterestPoints(cloud_orig);
	interestPoints = interestPointsScene.getInterestPoints();

	std::list <InterestPointsScene::PointInterest>::const_iterator it;
	for (it = interestPoints.begin() ; it != interestPoints.end() ; it++)
	{
		float dist = getDistance(*it);

		if(dist > 0.1)
		{
			Pblob instant(cloud_orig);

			addPoints2Plob(it->i, it->j, cloud, dist , instant);

			if(instant.getNPoints() > min_people_points_ && instant.getNPoints() < max_people_points_)
				instants.push_back(instant);
		}
	}
}

/**
 * \brief Function to merge X-Y overlapping plobs
 * \param[inout]	plobs	Vector of plobs to merge
 *
 * Merging plobs near, resulting of the plobs creation
 */
void
PlobPerception::reducePlobs(std::vector< Pblob >& instants)
{
	bool changed = true;

	while(changed && !instants.empty())
	{
		changed = false;
		std::vector< Pblob >::iterator it1, it2;

		for(it1 = instants.begin(); it1 != instants.end(); ++it1)
		{
			if(changed) break;

			for(it2 = instants.begin(); it2 != instants.end(); ++it2)
			{
				if(it1 == it2) continue;
				if(it1->near(*it2))
				{
					it1->merge(*it2);
					it2 = instants.erase(it2);
					changed = true;
					break;
				}
			}
		}
	}
}


bool PlobPerception::isFalsePerson(float sx , float sy , float sz)
{
	return  (sx > max_people_size_) || (sy > max_people_size_) || (sx <= min_people_size_) || (sy <= min_people_size_) || (KTE_Z * sy > sz) || (KTE_Z * sx > sz);
}

/**
 * \brief Check and remove the plobs with non-human dimesions
 * \param[inout]	plobs	Vector of plobs to check
 */
void
PlobPerception::checkPlobsSize(std::vector < Pblob > & instants)
{
	std::vector < Pblob > :: iterator it = instants.begin();

	while(it != instants.end())
	{
		float sx, sy, sz;

		sx = (it->getXmax() - it->getXmin());
		sy = (it->getYmax() - it->getYmin());
		sz = (it->getZmax() - it->getZmin());

		if( isFalsePerson(sx , sy , sz) )
			it = instants.erase(it);
		else
			++it;
	}
}


bool
PlobPerception::isSingularsPerson (const Pblob& instant)
{
	Head head (instant);
//	Legs legs (instant);

//	legs.setLegs();

//	publish_points_dep(legs.getLegs());
//	publish_points_dep(arms.getArms());
//	publish_points_dep(head.getHead());
//	if (legs.hasLegs() && head.isHeadDimensions()) {
//		publish_points_dep(legs.sgetLegs());
//		legs.getHistogram().print();
//		legs.similarity(legs_);
//	}
	return head.isHead();
}

void
PlobPerception::singularities (std::vector <Pblob> & instants)
{
	std::vector < Pblob > :: iterator it = instants.begin();

	while (it != instants.end())
	{
		if (isSingularsPerson(*it))
			it++;
		else
			it = instants.erase(it);
	}
//	fprintf(stderr , "\nSingularities finished...\n");
}


/**
 * \brief Cluod callback
 * \param[in]	rgbd	The new cloud
 *
 * The result of the processing is the vector of new plobs.
 */
void
PlobPerception::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& rgbd)
{

	try {

		sensor_msgs::PointCloud2 rgbd_bf;

		// If we are not able to pass base foot print image we can't process the point cloud return from handler.
		// in this code is necessary have base foot print running.
		pcl_ros::transformPointCloud(workingFrameId_, *rgbd, rgbd_bf, tfListener_); // This sentence can get the catch...

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_orig(new pcl::PointCloud<pcl::PointXYZRGB>);

		pcl::fromROSMsg(rgbd_bf, *cloud);
		pcl::fromROSMsg(rgbd_bf, *cloud_orig);

		if(cloud->empty()) return;

		std::vector < Pblob > instants;

		setInstantsFromInterestPoints(cloud , cloud_orig , instants);

//		tracePlobs(instants);

		checkPlobsSize(instants);
		reducePlobs(instants);
		singularities(instants);

//		tracePlobs(instants);

		pairPlobs(instants, permanents_);
		publish_plobs(permanents_);

		/* Publicamos una segunda nube de puntos con los puntos de interes de la escena un topic por pblob. */

//		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcrgb_out(new pcl::PointCloud<pcl::PointXYZRGB>);

//		sensor_msgs::PointCloud2 cloud_out;
//		pcl::toROSMsg(*pcrgb_out , cloud_out);

//		cloud_out.header.frame_id = rgbd_bf.header.frame_id;
//		pclPub.publish(cloud_out);

		instants.clear();

	} catch(tf::TransformException& ex) {

		ROS_ERROR_STREAM( "PairPlobs : Transform error of sensor data: " << ex.what() << ", quitting callback");
		return;

	}
}

void
PlobPerception::setTableSims (Eigen::MatrixXf& simTable , const std::vector < Pblob >& instant, const std::vector < Pblob >& permanent)
{
	float histo_sim = -1.0;
	float dist_sim = -1.0;
	float factor_dist = -1.0;

	for(int i = 0; i < instant.size(); i++)
	{
		for(int j = 0; j < permanent.size(); j++)
		{
			histo_sim = instant[i].similarity(permanent[j]);

			dist_sim = sqrt( ((permanent[j].getX() - instant[i].getX())*(permanent[j].getX() - instant[i].getX())) +
						  (permanent[j].getY() - instant[i].getY())*(permanent[j].getY() - instant[i].getY()));

			if (!std::isnan(histo_sim))
				simTable(i , j) = histo_sim; // Table of similarities between object based on color of this objects.
		}
	}
}

/*
 * Not finished yet.
 */
void
PlobPerception::printTableSims (const Eigen::MatrixXf& simTable , int sizePerm , int sizeInst)
{
	std::cout << std::endl;
	std::cout << "PairPlobs : simTable :\t" << std::endl;
	std::cout << std::endl;
	for (int i = 0; i < sizePerm; ++i) {
		const Eigen::IOFormat fmt(6, Eigen::AutoAlign, "\t" , "" , "" , "" , "" , "");
		std::cout << simTable.transpose().format(fmt) << std::endl;
	}
	std::cout << std::endl;
}


void
PlobPerception::update(std::vector < Pblob >& permanent , int id , const Pblob& instant)
{
//	fprintf(stderr, "PairPlobs : Updated permanent plob with Id : %d\n", id);
	permanent[id] = instant;
	permanent[id].updated_ts_ = ros::Time::now();
	countNumsFramesTrack++;
}

void
PlobPerception::add(std::vector < Pblob >& permanent , const Pblob& instant)
{
	Pblob newPlob(instant);

//	fprintf(stderr, "PairPlobs : Added new plob instant like permanent plob\n");

	newPlob.detected_ts_ = ros::Time::now();
	newPlob.updated_ts_  = ros::Time::now();

//	fprintf(stderr , "Site : (%lf , %lf) \n" , newPlob.getX() , newPlob.getY());
//	fprintf(stderr , "Size : %d \n" , newPlob.getSize());

	permanent.push_back(newPlob);
}

void
PlobPerception::erase(std::vector < Pblob >& permanent)
{
	std::vector< Pblob >::iterator it = permanent.begin();

//	fprintf (stderr , "PairPlobs : Check times and erase old trackers...\n");

	while (it != permanent.end())
	{
		if((ros::Time::now() - it->updated_ts_).toSec() > MAX_TIME)
		{
			it = permanent.erase(it);
//			fprintf(stderr, "PairPlobs : ==================\n");
//			fprintf(stderr, "PairPlobs : Drop Permanent\n");
//			fprintf(stderr, "PairPlobs : ==================\n");
		} else
			it++;
	}
}


void
PlobPerception::pairPlobs(const std::vector < Pblob >& instant, std::vector < Pblob >& permanent)
{
	if(instant.empty()) return;

	if(!permanent.empty())
	{
		/**
		 * There are plobs in permanents plobs :) we have to compare the actuals plobs
		 * in the scene with the list of permanents plobs.									  |P0  |P1  |P2  |
		 *																					==|====|====|====|
		 *																				  	T0|0.91|0.73|0.65|
		 *																				  	==|====|====|====|
		 * Create matrix with table of similarity blobs in the image and blobs permanents.	T1|0.72|0.92|0.53|
		 *																					==|====|====|====|
		 *																					* simTable.
		 */
		Eigen::MatrixXf simTable(instant.size() , permanent.size());

		setTableSims(simTable , instant , permanent);
		printTableSims(simTable , static_cast<int> (permanent.size()) , static_cast<int> (instant.size()));

		for(int i = 0; i < instant.size(); i++)
		{
			float max = 0.0;
			int id = 0;

			for(int j = 0; j < permanent.size(); j++)
				/*
				 * Get the position in the table of similarities the max
				 * similarity in the table of this permanent plob with an instance blob.
				 */
			{
				if(max < simTable(i , j))
				{
					max = simTable(i , j);
					id = j;
				}
			}
			/*
			 * Compare with a threshold of similarities, if the threshold is passed
			 * the permanent is corrected with the actual else the plob is new and
			 * we insert a new plob in the list of permanents blobs. Decider ML.
			 */
			if(max > MINPROBABILITY && max < THRESHOLD)
			{
				/*
				 * This plob not coincide with anyone plob in my list of
				 * permanent plobs we have to create a new plob.
				 */
				add(permanent , instant[i]);
			}
			else if(max > THRESHOLD && max < MAXPROBABILITY)
			{
				/*
				 * This is possible the same plot, we must to update the position
				 * and time of this plob.
				 */
				update(permanent , id , instant[i]);
			}
		}
		erase(permanent);
	}
	else
	{
		/*
		 * We have not permanents plobs then we have to
		 * add the new plob like my first plob :)
		 * possibility the tracker.
		 */
		for(int it_inst = 0; it_inst<instant.size(); it_inst++)
			add(permanent , instant[it_inst]);

	}
//	tracePlobs(permanents_);
//	fprintf (stderr , "PairPlobs : Number permanents persons detected : %d \n" , (int) permanents_.size());
}
