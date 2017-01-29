/*
 * Arms.cpp
 *
 *  Created on: Jun 2, 2016
 *      Author: javi
 */

#include "people_tracker/Body.h"

Body::Body(const Pblob& plob) :
body_(),
arms_(),
numDistArms_(0),
Pblob::Pblob()
{
	siluete_ = plob.getSiluete();

	xmin_ = plob.getXmin();
	xmax_ = plob.getXmax();

	sAx_ = xmax_ - xmin_;

	ymin_ = plob.getYmin();
	ymax_ = plob.getYmax();

	sAy_ = (ymax_ - ymin_) / WIDTHDIVIDE;

	zmin_ = plob.getZmin();
	zmax_ = plob.getZmax();

	sAz_ = (zmax_ - zmin_) / HEIGHDIVIDE;
}

Body::~Body()
{
	arms_.clear();
	body_.clear();
}

bool
orderByYArmsLine(pcl::PointXYZRGB pointA , pcl::PointXYZRGB pointB)
{
	return pointA.y < pointB.y;
}

bool
Body::isSiteArmsZ (const pcl::PointXYZRGB& point) const
{
	return point.z > zmin_ + sAz_ && point.z < zmax_ - sAz_ && point.z > SIZEZARMSMIN && point.z < SIZEZARMSMAX;
}

bool
Body::isSiteArmsY (const pcl::PointXYZRGB& point) const
{
	return point.y < ymin_ + sAy_ || point.y > ymax_ - sAy_;
}

bool
Body::isSiteArms (const pcl::PointXYZRGB& point) const
{
	return isSiteArmsZ(point) && isSiteArmsY(point);
}

bool
Body::isLineArms (const pcl::PointXYZRGB& point , float delta)
{
	return ( point.z > (zmin_ + sAz_) + delta) && ( point.z < (zmin_ + sAz_) + delta + INCREASE) && isSiteArmsZ(point);
}

void
Body::getLineArms (std::list<pcl::PointXYZRGB>& line , float delta)
{
	std::list<pcl::PointXYZRGB>::iterator it;

	for (it = siluete_.begin() ; it != siluete_.end() ; it++) {
		if (isLineArms(*it , delta)) {
			line.push_back(*it);
			line.sort(orderByYArmsLine);
			arms_.push_back(*it);
			body_.push_back(*it);
			histogram_.add(*it);
		}
	}
}

void
Body::removeArms(std::list<pcl::PointXYZRGB>& line)
{
	std::list<pcl::PointXYZRGB>::iterator it = line.begin();

	while (it != line.end())
		it = line.erase(it);
}


bool
Body::checkArms(const std::list<pcl::PointXYZRGB>& line)
{
	int numPointsSepArms = 0;
	std::list<pcl::PointXYZRGB>::const_iterator actual , previous;

	previous = line.begin();
	for (actual = line.begin() ; actual != line.end() ; actual++) {
		float distance = Pblob::getDistance(*actual , *previous);
		if (distance > DISTANCEBETWEENBODYANDARM) {
			numPointsSepArms++;
		}
		previous = actual;
	}
	return numPointsSepArms > 0 && numPointsSepArms <= MAXPOINTSEPARATESBODY;
}

bool
Body::areArms() const
{
	return numDistArms_ >= MINIMALLINESTOARMS;
}

void
Body::setArms()
{
	std::list<pcl::PointXYZRGB> line;
	float delta = INITARMSDELTA;
	numDistArms_ = 0;
//	fprintf (stderr  , "\nSet ARMS TO ONE PLOB BEGIN!!!\n");
	while (numDistArms_ < MINIMALLINESTOARMS && delta < INITARMSDELTA + SCANZARMS){
		getLineArms(line , delta);
		if (line.size() > MINPOINTSTOAMRS && checkArms(line))
			++numDistArms_;
		removeArms(line);
		delta += INCREASE;
	}
	if (areArms())
		ROS_WARN("ARMS!!!\n");
	fprintf (stderr  , "\nSet ARMS TO ONE PLOB END!!! numDistArms_ : %d\n" , numDistArms_);
}

void
Body::print (const std::list<pcl::PointXYZRGB>& line)
{
	std::list<pcl::PointXYZRGB>::const_iterator it , previous;

	fprintf (stderr , "\n***********************************ARMS POINTS LIST PRINT STARTED IN LINE***********************************\n");
	previous = line.begin();
	fprintf (stderr , "Number points legs of this pblob : %d \n" , static_cast<int>(line.size()) );
	for (it = line.begin() ; it != line.end() ; it++) {
		float distance = Pblob::getDistance(*it , *previous);
		if (distance > DISTANCEBETWEENBODYANDARM)
			fprintf (stderr , "Point position and distance with next : (x:%lf , y:%lf , z:%lf) distance:%lf\n" , it->x , it->y , it->z , distance);
		previous = it;
	}
	fprintf (stderr , "\n***********************************ARMS POINTS LIST PRINT FINISHED IN LINE***********************************\n");
}

void
Body::print () const
{
	std::list<pcl::PointXYZRGB>::const_iterator it , previous;

	fprintf (stderr , "\n***********************************ARMS POINTS LIST PRINT STARTED***********************************\n");
	previous = arms_.begin();
	fprintf (stderr , "Number points legs of this pblob : %d \n" , static_cast<int>(arms_.size()) );
	for (it = arms_.begin() ; it != arms_.end() ; it++) {
		float distance = Pblob::getDistance(*it , *previous);
		fprintf (stderr , "Point position and distance with next : (x:%lf , y:%lf , z:%lf) distance:%lf\n" , it->x , it->y , it->z , distance);
		previous = it;
	}
	fprintf (stderr , "************Print Histogram of this legs**********");
//	histogram_.printB();
	fprintf (stderr , "**************************************************");
	fprintf (stderr , "\n***********************************ARMS POINTS LIST PRINT FINISHED***********************************\n");
}

