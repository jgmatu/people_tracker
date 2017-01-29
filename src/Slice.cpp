/*
 * Slice.cpp
 *
 *  Created on: May 18, 2016
 *      Author: javi
 */

#include "../include/people_tracker/Slice.h"

Slice::Slice() :
	histogram_(),
	n_(0),
	xmin_(std::numeric_limits<float>::max()),
	xmax_(std::numeric_limits<float>::min()),
	ymin_(std::numeric_limits<float>::max()),
	ymax_(std::numeric_limits<float>::min()),
	zmin_(std::numeric_limits<float>::max()),
	zmax_(std::numeric_limits<float>::min())
{
	;
}

Slice::~Slice()
{
	;
}


void
Slice::add (pcl::PointXYZRGB& point)
{
	pcl::PointXYZHSV pointhsv;
	PointXYZRGBtoXYZHSV(point, pointhsv);

	add(pointhsv);
}

void
Slice::add (pcl::PointXYZHSV& point)
{
	histogram_.add(point);
	n_++;

	xmax_ = std::max(xmax_, point.x);
	xmin_ = std::min(xmin_, point.x);
	ymax_ = std::max(ymax_, point.y);
	ymin_ = std::min(ymin_, point.y);
	zmax_ = std::max(zmax_, point.z);
	zmin_ = std::min(zmin_, point.z);
}

void
Slice::add (const HSVHistogram& histogram , float zmax  , float zmin , float ymax , float ymin , float xmax , float xmin , int n)
{
	histogram_.add(histogram);
	zmax_ = zmax;
	zmin_ = zmin;
	ymax_ = ymax;
	ymin_ = ymin;
	xmax_ = xmax;
	xmin_ = xmin;
	n_ = n;
}


void
Slice::print () const
{
	fprintf(stderr , "Histogram Slice : \n");
	fprintf(stderr , "====================\n");
//	histogram_.print();
	fprintf(stderr , "====================\n");

	fprintf(stderr , "====================\n");
	fprintf(stderr , "Attributes Slice :  \n");
	fprintf(stderr , "Points Slice : %d \n" , n_);
	fprintf(stderr , "Dimmension  X: xmin : %lf xmax : %lf \n" , xmin_ , xmax_);
	fprintf(stderr , "Dimmension  Y: ymin : %lf ymax : %lf \n" , ymin_ , ymax_);
	fprintf(stderr , "Dimmension  Z: zmin : %lf zmax : %lf \n" , zmin_ , zmax_);
	fprintf(stderr , "====================\n");



}
