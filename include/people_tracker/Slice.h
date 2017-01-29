/*
 * Slice.h
 *
 *  Created on: May 18, 2016
 *      Author: javi
 */


#ifndef SLICE_H_
#define SLICE_H_


#include "people_tracker/HSVHistogram.h"



class Slice {

public:

	Slice();
	virtual ~Slice();

	void add(pcl::PointXYZRGB& pointcolor);
	void add(pcl::PointXYZHSV& pointcolor);
	void add (const HSVHistogram& histogram , float zmax  , float zmin , float ymax , float ymin , float xmax , float xmin , int n);



	float getXmin() const { return xmin_; };
	float getXmax() const { return xmax_; };
	float getYmin() const { return ymin_; };
	float getYmax() const { return ymax_; };
	float getZmin() const { return zmin_; };
	float getZmax() const { return zmax_; };

	float getX() const { return (xmax_+xmin_) / 2.0; };
	float getY() const { return (ymax_+ymin_) / 2.0; };
	float getZ() const { return (zmax_+zmin_) / 2.0; };

	void print () const;


private:

	HSVHistogram histogram_;
	int n_;
	float xmin_, xmax_, ymin_, ymax_ , zmin_ , zmax_;

};

#endif /* INCLUDE_PEOPLE_TRACKER_SLICE_H_ */
