/*
 * Pblob.cpp
 *
 *  Created on: 28/04/2016
 *      Author: Paco
 */

#include "people_tracker/Pblob.h"

#include <limits>


void
Pblob::printInitialize ()
{
	fprintf(stderr,  "\n=============================================\n");
	fprintf(stderr,  "\nPrint trace Plob :) Create in constructor :).\n");
	fprintf(stderr,  "\n=============================================\n");

	fprintf(stderr , "\nHistogram Plob : \n");
	fprintf(stderr,  "\n=============================================\n");
//	histogram_.print();
	fprintf(stderr,  "\n=============================================\n");
	fprintf(stderr , "Values of x y and z. xmin : %lf , xmax : %lf , ymin : %lf , ymax : %lf , zmin : %lf , zmax : %lf " , xmin_ , xmax_ , ymin_  , ymax_ , zmin_ , zmax_);
	fprintf(stderr,  "\n=============================================\n");

	fprintf (stderr , "\nNumber of Points : %d\n" , npoints_);

	fprintf (stderr , "\nSlices : \n");
	fprintf (stderr , "==================\n");

//	for (int i = 0 ; i < SLICEN ; i++)
//		slices_[i]->print();

}

/*
 * This constructor is used for inherits classes to initialize the values
 * of one Pblob.
 */
Pblob::Pblob()
: indices_(), 	// Indices with vector of indices empty.s
  histogram_(), // Histogram initialize to zero.
  xmin_(std::numeric_limits<float>::max()),
  xmax_(FLOAT_MIN),
  ymin_(std::numeric_limits<float>::max()),
  ymax_(FLOAT_MIN),
  zmin_(std::numeric_limits<float>::max()),
  zmax_(FLOAT_MIN),
  npoints_(0),
  siluete_()
{
//	for (int i = 0 ; i < SLICEN ; i++)
//		slices_[i] = new Slice();
//	printInitialize();
}


Pblob::Pblob(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
: indices_(), 	// Indices with vector of indices empty.s
  histogram_(), // Histogram initialize to zero.
  xmin_(std::numeric_limits<float>::max()),
  xmax_(FLOAT_MIN),
  ymin_(std::numeric_limits<float>::max()),
  ymax_(FLOAT_MIN),
  zmin_(std::numeric_limits<float>::max()),
  zmax_(FLOAT_MIN),
  cloud_(cloud), // The blob in create with a point cloud associate the Pblob inside the scene.
  npoints_(0),
  siluete_()
{
//	for (int i = 0 ; i < SLICEN ; i++)
//		slices_[i] = new Slice();
//	printInitialize();
//	fprintf(stderr , "xmax : %lf , ymax : %lf , zmax : %lf \n" , xmax_ , ymax_ ,zmax_);
}


Pblob::~Pblob()
{
	siluete_.clear();
}

float
Pblob::getDistance (const pcl::PointXYZRGB& pointA , const pcl::PointXYZRGB& pointB) const
{
	return sqrt ( (pointA.x - pointB.x) * (pointA.x - pointB.x) + (pointA.y - pointB.y) * (pointA.y - pointB.y) + (pointA.z - pointB.z) * (pointA.z - pointB.z) );
}

float
Pblob::getDistance (const pcl::PointXYZRGB& point) const
{
	return sqrt ( (point.x) * (point.x) + (point.y) * (point.y) + (point.z) * (point.z) );
}

bool
Pblob::insidePblob (pcl::PointXYZRGB& point)
{
	return ((point.x >=  xmin_) && (point.x <=  xmax_)) &&
				((point.y >=  ymin_) && (point.y <=  ymax_)) &&
				((point.z >=  zmin_) && (point.z <=  zmax_));
}

/*
 * See if the point in the cloud is owns of this blob
 * calculating the distance between points.
 */
bool
Pblob::owns(const int pointIndex)
{
	pcl::PointXYZRGB *point= &(cloud_->at(pointIndex));

	if(std::isnan(point->x)) return false;

	if ( insidePblob(*point) ) return true;
	if ( fabs(point->x - xmax_) <= PBLOB_DIST_TOLERANCE ) return true;
	if ( fabs(point->x - xmin_) <= PBLOB_DIST_TOLERANCE ) return true;
	if ( fabs(point->y - ymax_) <= PBLOB_DIST_TOLERANCE ) return true;
	if ( fabs(point->y - ymax_) <= PBLOB_DIST_TOLERANCE ) return true;
	if ( fabs(point->z - zmax_) <= PBLOB_DIST_TOLERANCE ) return true;
	if ( fabs(point->z - zmax_) <= PBLOB_DIST_TOLERANCE ) return true;

	return false;
}


//float
//getDistance (pcl::PointXYZRGB pointA , pcl::PointXYZRGB pointB)
//{
//	return sqrt ( ( pointA.x - pointB.x ) * ( pointA.x - pointB.x ) +
//					( pointA.y - pointB.y ) * ( pointA.y - pointB.y ) +
//					( pointA.z - pointB.z ) * ( pointA.z - pointB.z ) ) ;
//}


/*
 * We insert the point inside the histogram of blob,
 * change the max point x y to new point if it is greater
 * than before.
 *
 * And set the pointIndex of blob to vector of index.
 */
void
Pblob::add(const int pointIndex)
{

	pcl::PointXYZRGB *point= &(cloud_->at(pointIndex));

	if(std::isnan(point->x)) return;

	indices_.push_back(pointIndex);

	xmax_ = std::max(xmax_, point->x);
	xmin_ = std::min(xmin_, point->x);
	ymax_ = std::max(ymax_, point->y);
	ymin_ = std::min(ymin_, point->y);
	zmax_ = std::max(zmax_, point->z);
	zmin_ = std::min(zmin_, point->z);

	npoints_++;
	siluete_.push_front(*point);

	histogram_.add(*point);


//	float zslice = zmax_ / (SLICEN - 1);
//	int pos = static_cast<int>(point->z/zslice);
//	slices_[pos].add(*point);
}

/*
 * Near in positions x and y is variable z constant
 * the robot not fly :).
 */
bool
Pblob::near(const Pblob& plob)
{
	float x1 = xmin_;
	float X1 = xmax_;
	float y1 = ymin_;
	float Y1 = ymax_;

	float x2 = plob.getXmin();
	float X2 = plob.getXmax();
	float y2 = plob.getYmin();
	float Y2 = plob.getYmax();

	bool inX, inY;

	inX = (fabs(x1-X2) < PBLOB_DIST_TOLERANCE) || (fabs(X1-x2) < PBLOB_DIST_TOLERANCE) ||
		  ((x2 < X1) && (x2>x1)) || ((X2 < X1) && (X2 > x1));

	inY = (fabs(y1-Y2) < PBLOB_DIST_TOLERANCE) || (fabs(Y1-y2) < PBLOB_DIST_TOLERANCE) ||
		  ((y2 < Y1) && (y2 > y1)) || ((Y2 < Y1) && (Y2 > y1));

	return inX && inY;
}

/*
 * See if the blob passed and the blob instantiate are in the same positions and
 * merge both blobs.
 */
void
Pblob::merge(Pblob& blob)
{
	xmin_ = std::min(xmin_, blob.getXmin());
	xmax_ = std::max(xmax_, blob.getXmax());
	ymin_ = std::min(ymin_, blob.getYmin());
	ymax_ = std::max(ymax_, blob.getYmax());
	zmin_ = std::min(zmin_, blob.getZmin());
	zmax_ = std::max(zmax_, blob.getZmax());
	npoints_ += blob.npoints_;

	histogram_.add(blob.getHistogram()); // The points of histogram in one blob add to the other histogram of blob.. C = A + B (HS) Matrix.

	indices_.insert(indices_.end(), blob.getIndices().begin(), blob.getIndices().end());
}


void
Pblob::operator=(const Pblob& other)
{
	this->cloud_ 	 = other.cloud_;
	this->histogram_ = other.histogram_;
	this->indices_   = other.indices_;
	this->xmax_		 = other.xmax_;
	this->xmin_		 = other.xmin_;
	this->ymax_		 = other.ymax_;
	this->ymin_      = other.ymin_;
	this->zmax_ 	 = other.zmax_;
	this->zmin_		 = other.zmin_;
	this->siluete_	 = other.siluete_;
	this->npoints_	 = other.npoints_;
}


visualization_msgs::Marker*
Pblob::getRectangleTemplate() const
{
	visualization_msgs::Marker *marker;
	geometry_msgs::Point point;

	marker = new visualization_msgs::Marker;

	marker->header.frame_id = "/base_foorprint";
	marker->header.stamp = ros::Time();
	marker->ns = "templatePlob";
	marker->id = 0;
	marker->type = visualization_msgs::Marker::LINE_LIST;
	marker->action = visualization_msgs::Marker::ADD;
	marker->pose.position.x = getX();
	marker->pose.position.y = getY();
	marker->pose.position.z = getZ();
	marker->pose.orientation.x = 0.0;
	marker->pose.orientation.y = 0.0;
	marker->pose.orientation.z = 0.0;
	marker->pose.orientation.w = 1.0;
	marker->scale.x = 1.0;
	marker->scale.y = 0.0;
	marker->scale.z = 0.0;
	marker->color.a = 1.0; // Don't forget to set the alpha!
	marker->color.r = 0.0;
	marker->color.g = 1.0;
	marker->color.b = 0.0;

	point.x = xmin_;
	point.y = getY();
	point.z = zmax_;
	marker->points.push_back(point);

	point.x = xmax_;
	point.y = getY();
	point.z = zmax_;
	marker->points.push_back(point);
	marker->points.push_back(point);

	point.x = xmax_;
	point.y = getY();
	point.z = zmin_;
	marker->points.push_back(point);
	marker->points.push_back(point);

	point.x = xmin_;
	point.y = getY();
	point.z = zmin_;
	marker->points.push_back(point);
	marker->points.push_back(point);

	point.x = xmin_;
	point.y = getY();
	point.z = zmax_;
	marker->points.push_back(point);

	return marker;
}

void
Pblob::printPblob() const
{
	fprintf (stderr , "\n******* Pblob Printed ********\n");
	fprintf (stderr , "Location in the space inside the scene : (%lf , %lf , %lf) \n" , this->getX() , this->getY() , this->getZ());
	fprintf (stderr , "Dimesions of pblob : sx : %lf , sy : %lf , sz : %lf\n" ,
			this->getXmax() - this->getXmin() ,
			this->getYmax() - this->getYmin() ,
			this->getZmax() - this->getZmin());

	fprintf (stderr , "Number of points that's define the pblob : %d\n" , this->npoints_ );
	fprintf (stderr , "\n ****************************\n");
}


void
Pblob::printPoints () const
{
	std::list<pcl::PointXYZRGB>::const_iterator it;

	fprintf (stderr , "Number points border of this pblob : %d \n" , static_cast<int>(siluete_.size()) );
	for (it = siluete_.begin() ; it != siluete_.end() ; it++)
		fprintf (stderr , "Point position and distance with next : (x:%lf , y:%lf , z:%lf)\n" , it->x , it->y , it->z);
}


//void
//Pblob::printSlices() const
//{
//	fprintf (stderr , "Number of Slices : %d \n" , SLICEN);
//	fprintf (stderr , "======================\n");
//	for (int i = 0 ; i < SLICEN ; i++)
//	{
//		fprintf(stderr , "Slice%d : \n" , i);
//		slices_[i]->print();
//		fprintf(stderr , "\n");
//	}
//	fprintf (stderr , "======================\n");
//
//}
