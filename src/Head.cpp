/*
 * Head.cpp
 *
 *  Created on: May 30, 2016
 *      Author: javi
 */

#include "people_tracker/Head.h"

Head::Head(const Pblob& plob) :
head_()
{
	siluete_ = plob.getSiluete();

	ymax_ = plob.getYmax();
	ymin_ = plob.getYmin();

	zmax_ = plob.getZmax();
}

Head::~Head()
{
	head_.clear();
}

bool
Head::isPointLimitZ(const pcl::PointXYZRGB point)
{
	return point.z > (zmax_ - BORDER);
}


/**
 * METHOD PUBLIC SET ATTRIBUTES OF HEAD IN ONE SILUETE
 *
 * Call this method just later construct one instance of head.
 *
 * This method is public is used to define the attributes
 * of head of one Pblob once created...
 */
bool
Head::isHead ()
{

	float sizeHmax = std::numeric_limits<float>::min() + FLOAT_MIN;
	fsloat sizeHmin = std::numeric_limits<float>::max();
	float sy = ( ymax_ - ymin_ ) , sHy = 0.0;

//	fprintf(stderr  , "sizeHmax : %lf \n"  , sizeHmax);
//	fprintf(stderr  , "sizeHmin : %lf \n"  , sizeHmin);

	std::list<pcl::PointXYZRGB>::const_iterator it;
	for (it = siluete_.begin() ; it != siluete_.end() ; it++)
	{
		if (isPointLimitZ(*it))
		{
			sizeHmin = std::min(sizeHmin, it->y);
			sizeHmax = std::max(sizeHmax, it->y);
			head_.push_back(*it);
		}
	}

	if ((int)head_.size() > 1) {
		sHy = sizeHmax - sizeHmin;
	}

//	fprintf(stderr  , "sizeHmax : %lf \n"  , sizeHmax);
//	fprintf(stderr  , "sizeHmin : %lf \n"  , sizeHmin);

//	print(sHy , sy , (int)head_.size());

	return  sHy > MINSIZE && sHy < MAXSIZE && sHy > 0 && sHy < (sy - BODY) && (int)head_.size() < MAXPOINTSHEAD;
}

void
Head::print (float sHy , float sy , int nhead) const
{
	fprintf (stderr , "\n****************** PRINTING HEAD BEGIN VALUES ****************** \n");
	fprintf (stderr , "Size Dimensions of head relative to pblob : sy : %lf \n" , sy);
	fprintf (stderr , "Number points of head : %d \n" , nhead);
	fprintf (stderr , "Size Head in Y : %lf \n" , sHy);
	fprintf (stderr , "****************** PRINTING HEAD FINAL VALUES ****************** \n");
}
