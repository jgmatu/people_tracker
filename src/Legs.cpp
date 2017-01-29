/*
 * Legs.cpp
 *
 *  Created on: May 31, 2016
 *      Author: javi
 */

#include "people_tracker/Legs.h"


Legs::Legs(const Pblob& pblob) :
legs_(),
nlegs_(0),
legsCount_(0),
Pblob::Pblob()
{
	siluete_ = pblob.getSiluete();
	npoints_ = pblob.getNPoints();
	xmin_ = pblob.getXmin();
	xmax_ = pblob.getXmax();
	ymin_ = pblob.getYmin();
	ymax_ = pblob.getYmax();
	zmin_ = pblob.getZmin();
	zmax_ = pblob.getZmax();

	sLx_  =    xmax_ - xmin_;
	sLy_  =  ( ymax_ - ymin_ ) / WIDTHDIVIDE;
	sLz_  =  ( zmax_ - zmin_ ) / HEIGHDIVIDE;
}

Legs::Legs() :
legs_(),
nlegs_(0),
legsCount_(0),
Pblob::Pblob()
{
	sLx_  = xmax_ - xmin_;
	sLy_  =  ( ymax_ - ymin_ ) / WIDTHDIVIDE;
	sLz_  =  ( zmax_ - zmin_ ) / HEIGHDIVIDE;
}

Legs::~Legs()
{
	legs_.clear();
}


bool
Legs::isDistance (pcl::PointXYZRGB pointA , pcl::PointXYZRGB pointB)
{
	return getDistance(pointA , pointB) > MINLEGSDISTANCE;
}



bool
Legs::isSiteLegs(pcl::PointXYZRGB pointA)
{
	return pointA.z < SITELEGS;
}

bool
Legs::isPointLimitYLegs(pcl::PointXYZRGB point)
{
	return point.y > ymin_ + sLy_ && point.y < ymax_ - sLy_;
}

bool
Legs::isPointLimitZLegs(pcl::PointXYZRGB point)
{
	return point.z > zmax_ - sLz_;
}

/*
 * This function is used to test legs negated...
bool
Pblob::isLegs(pcl::PointXYZRGB point , float delta)
{
	return point.z > zmax_ - delta - 0.01 && point.z < zmax_ - delta;
}
*/

bool
Legs::isLine (pcl::PointXYZRGB point , float delta)
{
	return point.z < zmin_ + delta + INCREASE && point.z > zmin_ + delta;
}

bool
Legs::isLegs(pcl::PointXYZRGB point , float delta)
{
	return isLine(point , delta) /*  && isPointLimitZLegs(point) && isPointLimitYLegs(point)*/;
}


bool
orderByYLegsLine (pcl::PointXYZRGB pointA, pcl::PointXYZRGB pointB)
{
	return pointA.y > pointB.y;
}

bool
orderByZ (pcl::PointXYZRGB pointA, pcl::PointXYZRGB pointB)
{
	return pointA.z > pointB.z;
}

void
Legs::getLineLegs(float delta , std::list < pcl::PointXYZRGB > & line)
{
	std::list<pcl::PointXYZRGB>::iterator it;

	for (it = siluete_.begin() ; it != siluete_.end() ; it++) {
		if (isLine(*it , delta)) {

			line.push_back(*it);
			line.sort(orderByYLegsLine);

			legs_.push_back(*it);
			histogram_.add(*it);
		}
	}

//	fprintf(stderr , "Line in Z Threshold : (%lf , %lf ) \n" , zmin_ + delta , zmin_ + delta + 0.01);
//	fprintf(stderr , "Value of z line : %lf \n" , zmin_ + delta);
//	fprintf(stderr , "Num points Line : %d  \n" , static_cast<int>(line.size()));
}



bool
Legs::areLegs (pcl::PointXYZRGB pointA , pcl::PointXYZRGB pointB)
{
	return  isDistance(pointA , pointB) && isSiteLegs(pointA);
}


bool
Legs::checkLegLine(const std::list<pcl::PointXYZRGB>& line)
{
	std::list<pcl::PointXYZRGB>::const_iterator actual , previous;
	int distance = 0;
	int numPointsLine = 0;

	previous = line.begin();
	for (actual = line.begin() ; actual != line.end() ; actual++) {

		if (areLegs(*actual , *previous)) {
//			fprintf (stderr , "This line are legs in points  : point.y : %lf , previous.y : %lf\n" , point.y , previous.y);
//			fprintf (stderr , "This line are legs in points  : (%lf , %lf , %lf) \n" , point.x , point.y , point.z);
			distance++;
		}
		previous = actual;
	}
//	fprintf(stderr , "Points of line : %d\n" , static_cast<int>(line.size()));
//	fprintf(stderr , "Num Distances  : %d\n" , distance);
	return distance > 0 && distance <= MAXPOINTSSEPARATESLINE;
}

void
Legs::removeLegs (std::list<pcl::PointXYZRGB>& line)
{
	std::list<pcl::PointXYZRGB>::iterator it = line.begin();

	while (it != line.end())
		it = line.erase(it);
}


/*
 * METHOD PUBLIC CHECK IF THE SILUETE HAVE THE CARACTERISTICS OF LEGS.
 *
 * Call this function is not possible if you not call first at constructor
 * and just later the public method hasLegs is very important follow this
 * steps to call to this function outside of class.
 */
bool
Legs::hasLegs ()
{
	return legsCount_ > MINLINELEGSCONSIDERLEGS;
}

/*
 * METHOD PUBLIC SET ATTRIBUTES OF LEGS IN ONE SILUETE
 *
 * Call this method just later construct one instance of legs.
 *
 * This method is public is used to define the attributes
 * of legs of one Pblob once created...
 *
 */
void
Legs::setLegs ()
{
	float delta = 0.00;
	std::list<pcl::PointXYZRGB> line;

	legsCount_ = 0;
	while (delta < SCANZSUPERFICE) {
		/*
		 * Scan down part of the pblob in search of legs 30 cm
		 * from down to up.
		 */
		getLineLegs(delta , line);

		if (static_cast<int>(line.size()) > MINPOINTSLINE &&  checkLegLine(line) )
			legsCount_++;

		removeLegs(line);
		delta += INCREASE;
	}

	if (hasLegs())
		ROS_ERROR("LEGS!\n");
}

float sLx_ , sLy_ , sLz_;
int  nlegs_ , legsCount_;


void
Legs::operator=(const Legs& other)
{
	Pblob::operator =(other);
	this->sLx_       = other.sLx_;
	this->sLy_      = other.sLy_;
	this->sLz_       = other.sLz_;
	this->nlegs_     = other.nlegs_;
	this->legs_      = other.legs_;
}

void
Legs::print (const std::list<pcl::PointXYZRGB>& line)
{
	std::list<pcl::PointXYZRGB>::const_iterator actual , previous;

	fprintf (stderr , "\n***********************************LEGS POINTS LIST PRINT STARTED***********************************\n");

	previous = line.begin();
	fprintf (stderr , "Number points legs of this pblob : %d \n" , static_cast<int>(line.size()) );
	for (actual = line.begin() ; actual != line.end() ; actual++) {

		float distance = Pblob::getDistance(*actual , *previous);

		if (distance > MINLEGSDISTANCE || true)
			fprintf (stderr , "Point position and distance with next : (x:%lf , y:%lf , z:%lf) distance:%lf\n" , actual->x , actual->y , actual->z , distance);

		previous = actual;
	}
	fprintf (stderr , "\n***********************************LEGS POINTS LIST PRINT FINISHED***********************************\n");
}

void
Legs::print () const
{
	std::list<pcl::PointXYZRGB>::const_iterator actual , previous;

	fprintf (stderr , "\n***********************************LEGS POINTS LIST PRINT STARTED***********************************\n");

	previous = legs_.begin();
	fprintf (stderr , "Number points legs of this pblob : %d \n" , static_cast<int>(legs_.size()) );

	for (actual = legs_.begin() ; actual != legs_.end() ; actual++) {

		float distance = Pblob::getDistance(*actual , *previous);

		if (distance > MINLEGSDISTANCE)
			fprintf (stderr , "Point position and distance with next : (x:%lf , y:%lf , z:%lf) distance:%lf\n" , actual->x , actual->y , actual->z , distance);

		previous = actual;

	}

	fprintf (stderr , "************Print Histogram of this legs**********");
//	histogram_.printL();
	fprintf (stderr , "**************************************************");

	fprintf (stderr , "\n***********************************LEGS POINTS LIST PRINT FINISHED***********************************\n");
}
