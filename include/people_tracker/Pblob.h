/*
 * Pblob.h
 *
 *  Created on: 28/04/2016
 *      Author: paco
 */

#ifndef PBLOB_H_
#define PBLOB_H_

#include <ros/ros.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "visualization_msgs/Marker.h"

#include "people_tracker/HSVHistogram.h"
#include "people_tracker/Slice.h"

#define PBLOB_DIST_TOLERANCE 0.10 // Distance tolerance one object in the scene is owned of the same PBLOB. 20 cm.
#define FLOAT_MIN  -340282346638528859811704183484516925440.0

class Pblob {

public:

	Pblob(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
	Pblob();

	virtual ~Pblob();

	inline bool owns(const int pointIndex);
	void add(const int pointIndex);

	bool near(const Pblob& blob);
	void merge(Pblob& blob);

	float getXmin() const { return xmin_; };
	float getXmax() const { return xmax_; };
	float getYmin() const { return ymin_; };
	float getYmax() const { return ymax_; };
	float getZmin() const { return zmin_; };
	float getZmax() const { return zmax_; };

	float getX() const { return (xmax_ + xmin_) / 2.0; };
	float getY() const { return (ymax_ + ymin_) / 2.0; };
	float getZ() const { return (zmax_ + zmin_) / 2.0; };

	int getNPoints() const { return npoints_; };

	std::list<pcl::PointXYZRGB> getSiluete() const { return siluete_; };

	int getSize()   const { return indices_.size(); };
	std::vector<int>& getIndices() { return indices_; };

	const HSVHistogram& getHistogram() const { return histogram_; };
	float similarity(const Pblob& other) const {return histogram_.similarity(other.getHistogram());};

	void operator=(const Pblob& other);

	visualization_msgs::Marker* getRectangleTemplate() const;

	ros::Time detected_ts_;
	ros::Time updated_ts_;

	void printPoints () const;
	void printPblob () const;
	//	void printSlices () const;

private:

	static const int SLICEN = 2;

	bool insidePblob (pcl::PointXYZRGB& point);
	void printInitialize ();
	void addFalsePoints (const pcl::PointXYZRGB point);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;

	/*
	 * The pblob have a vector of index a point cloud whose represent
	 * the scene where is the pblob, and a histogram whose represent the pblob.
	 */
	std::vector<int> indices_; // This vector what represent?

	//	Slice *slices_ [SLICEN];

protected :

	float getDistance (const pcl::PointXYZRGB& point) const;
	float getDistance (const pcl::PointXYZRGB& pointA , const pcl::PointXYZRGB& pointB) const;

	std::list<pcl::PointXYZRGB> siluete_;
	HSVHistogram histogram_;

	/*
	 * Position x and y max of blob. borders of blob.
	 */
	float xmin_, xmax_, ymin_, ymax_ , zmin_ , zmax_;
	/*
	 * Number of points of one plob.
	 */
	int   npoints_;
};

#endif /* PBLOB_H_ */
