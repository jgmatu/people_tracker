/*
 * HSVHistogram.cpp
 *
 *  Created on: 28/04/2016
 *      Author: Paco
 */

#include "people_tracker/HSVHistogram.h"

void
HSVHistogram::inizialize()
{
	for(int i=0; i<SIZEH; i++) {
		for(int j=0; j<SIZES; j++) {
			HS_P_[i][j] = 0;
//			HS_H_[i][j] = 0;
//			HS_B_[i][j] = 0;
//			HS_L_[i][j] = 0;

			HSPROB_P_[i][j] = 0.0;
//			HSPROB_H_[i][j] = 0-0;
//			HSPROB_B_[i][j] = 0-0;
//			HSPROB_L_[i][j] = 0.0;
		}
	}
}

/*
 * Create a new histogram initialize to 0. :).
 */
HSVHistogram::HSVHistogram()
: p_(0),
//  h_(0),
//  b_(0),
//  l_(0),
  dep_()
{
	inizialize();
}

/*
 * Create a histogram from one histogram.
 */
HSVHistogram::HSVHistogram(const HSVHistogram& other)
: p_(other.p_)
//  h_(other.h_),
//  b_(other.b_)
//  l_(other.l_)
{
	dep_ = other.getDep();

	memcpy(HS_P_, other.HS_P_, SIZEH*SIZES*sizeof(int));
	memcpy(HSPROB_P_ , other.HSPROB_P_ , SIZEH*SIZES*sizeof(float));

//	memcpy(HS_H_, other.HS_H_, SIZEH*SIZES*sizeof(int));
//	memcpy(HSPROB_H_ , other.HSPROB_H_ , SIZEH*SIZES*sizeof(float));

//	memcpy(HS_B_, other.HS_B_, SIZEH*SIZES*sizeof(int));
//	memcpy(HSPROB_B_ , other.HSPROB_B_ , SIZEH*SIZES*sizeof(float));

//	memcpy(HS_L_, other.HS_L_, SIZEH*SIZES*sizeof(int));
//	memcpy(HSPROB_L_ , other.HSPROB_L_ , SIZEH*SIZES*sizeof(float));

}

HSVHistogram::~HSVHistogram()
{
	dep_.clear();
}

/*
 * Copy the numbers of points whose represent the histogram
 * and copy the histogram matrix 20*10 in other site of memory
 * like a new histogram.
 */

HSVHistogram&
HSVHistogram::operator=(const HSVHistogram& other)
{
	inizialize();

	p_ = other.p_;
	memcpy(HS_P_, other.HS_P_, SIZEH*SIZES*sizeof(int));
	memcpy(HSPROB_P_ , other.HSPROB_P_ , SIZEH*SIZES*sizeof(float));

//	h_ = other.h_;
//	memcpy(HS_H_, other.HS_H_, SIZEH*SIZES*sizeof(int));
//	memcpy(HSPROB_H_ , other.HSPROB_H_ , SIZEH*SIZES*sizeof(float));

//	b_ = other.b_;
//	memcpy(HS_B_, other.HS_B_, SIZEH*SIZES*sizeof(int));
//	memcpy(HSPROB_B_ , other.HSPROB_B_ , SIZEH*SIZES*sizeof(float));

//	l_ = other.l_;
//	memcpy(HS_L_, other.HS_L_, SIZEH*SIZES*sizeof(int));
//	memcpy(HSPROB_L_ , other.HSPROB_L_ , SIZEH*SIZES*sizeof(float));

	setProbs();
	return *this;
}


void
HSVHistogram::setPositionXYZ (const pcl::PointXYZRGB& pointSource , pcl::PointXYZHSV& pointDest)
{
	pointDest.x = pointSource.x;
	pointDest.y = pointSource.y;
	pointDest.z = pointSource.z;
}

void
HSVHistogram::setPositionXYZ (const pcl::PointXYZHSV& pointSource , pcl::PointXYZRGB& pointDest)
{
	pointDest.x = pointSource.x;
	pointDest.y = pointSource.y;
	pointDest.z = pointSource.z;
}
/*
bool
HSVHistogram::isHead (const pcl::PointXYZHSV& pointcolor) const
{
	return pointcolor.z > MINHEAD && pointcolor.z < MAXHEAD;
}
*/

/*
bool
HSVHistogram::isBody (const pcl::PointXYZHSV& pointcolor) const
{
	return pointcolor.z > MINBODY && pointcolor.z < MAXBODY;
}
*/

/*
bool
HSVHistogram::isLegs (const pcl::PointXYZHSV& pointcolor) const
{
	return  pointcolor.z > MINLEGS && pointcolor.z < MAXLEGS;
}
*/
/*
void
HSVHistogram::depHist (pcl::PointXYZHSV pointcolor)
{
	pcl::PointXYZRGB pointrgb;

	PointXYZHSVtoXYZRGB(pointcolor , pointrgb);
	setPositionXYZ(pointcolor , pointrgb);

	dep_.push_back(pointrgb);
}
*/

/*
 * Convert the point to hsv values and see if the values
 * of hsv are inside the range of values. 0..20 in hue and 0..10
 * in saturation.
 */
//bool
//HSVHistogram::inRange(pcl::PointXYZRGB& pointcolor) const
//{
//	pcl::PointXYZHSV pointcolorhsv;

//	PointXYZRGBtoXYZHSV(pointcolor, pointcolorhsv);

//	return inRange(pointcolorhsv);
//}

/*
 * See if the point in hue value and saturation value
 * are inside the range of values. 0..20 in hue and 0..10
 * in saturation.
 */
//bool
//HSVHistogram::inRange(pcl::PointXYZHSV& pointcolor) const
//{
//	int idxH, idxS;

//	idxH = pointcolor.h/(MAXH/SIZEH);
//	idxS = pointcolor.s/(MAXS/SIZES);

//	return HS_P_[idxH][idxS] >= 0;
//}

void
HSVHistogram::setProbs()
{
	int p = 0 /* , h = 0 , b = 0 , l = 0*/;

	for(int i = 0 ; i < SIZEH ; i++) {
		for(int j = 0 ; j < SIZES ; j++) {
			p += HS_P_[i][j];
//			h += HS_H_[i][j];
//			b += HS_B_[i][j];
//			l += HS_L_[i][j];
		}
	}

	for(int i = 0 ; i < SIZEH ; i++) {
		for(int j = 0 ; j < SIZES ; j++) {

			if (p != 0)
				HSPROB_P_[i][j] =  static_cast<float> (HS_P_[i][j]) / float(p);

//			if (h != 0)
//				HSPROB_H_[i][j] =  static_cast<float> (HS_H_[i][j]) / float(h);

//			if (b != 0)
//				HSPROB_B_[i][j] =  static_cast<float> (HS_B_[i][j]) / float(b);

//			if (l != 0)
//				HSPROB_L_[i][j] =  static_cast<float> (HS_L_[i][j]) / float(l);

		}
	}
	p_ = p;
//	h_ = h;
//	b_ = b;
//	l_ = l;
}

/*
 * Add a new point to the histogram with a point cloud
 * in RGB parameters.
 */
void
HSVHistogram::add(pcl::PointXYZRGB& pointcolor)
{
	pcl::PointXYZHSV pointcolorhsv;

	PointXYZRGBtoXYZHSV(pointcolor, pointcolorhsv);
	setPositionXYZ(pointcolor , pointcolorhsv);
	add(pointcolorhsv);
}

void
HSVHistogram::add(pcl::PointXYZHSV& pointcolor)
{
	if(pointcolor.v < MINVALIDV || pointcolor.v > MAXVALIDV || pointcolor.s < MINVALIDS)
		return;

	int idxH, idxS;

	idxH = pointcolor.h/(MAXH/SIZEH);  // 360.0 / 20.0 18.0 -> Range in 0 .. 20 of hue... steps... :).
	idxS = pointcolor.s/(MAXS/SIZES);  // 1 / 10.0 -> Range 0 .. 10 of saturation... steps... :).

	HS_P_[idxH][idxS]++;
	p_++;

/*
	if (isHead(pointcolor)) {

		HS_H_[idxH][idxS]++;
		h_++;

	} else  if ( isBody(pointcolor)) {

		HS_B_[idxH][idxS]++;
		b_++;
		depHist(pointcolor);

	} else if ( isLegs(pointcolor)) {

		HS_L_[idxH][idxS]++;
		l_++;
		depHist(pointcolor);

	}
*/
	setProbs(); // SetProbs point per point.
}


void
HSVHistogram::add(const HSVHistogram& histogram)
{

	for(int i = 0; i < SIZEH; i++) {
		for(int j = 0; j < SIZES; j++) {
			HS_P_[i][j] += histogram.getHS_P_(i , j);
//			HS_H_[i][j] += histogram.getHS_H_(i, j);
//			HS_B_[i][j] += histogram.getHS_B_(i, j);
//			HS_L_[i][j] += histogram.getHS_L_(i, j);
		}
	}
	p_ += histogram.getP();
//	h_ += histogram.getH();
//	b_ += histogram.getB();
//	l_ += histogram.getL();

	setProbs(); // Set probs to all the HSPROB in the add histogram to my actual histogram.
}



/*
 * Compare to histogram this and other to see if there are similarities
 * in the two histograms.
 */
float
HSVHistogram::similarity(const HSVHistogram& histogram) const
{
	float simplob = 0.0 , simhead = 0.0 , simbody = 0.0 , simlegs = 0.0;
	float sim = 0.0;


	for (int i = 0 ; i < SIZEH ; i++) {
		for (int j = 0 ; j < SIZES ; j++) {
			if (simhead > 1.1 || simbody > 1.1|| simlegs > 1.1 || simplob > 1.1) {
//				std::cerr<<std::endl;
//				fprintf(stderr , "Value of HSPROB_B_ %lf , Value of getHSPROB_B_ : %lf , value simbody : %lf \n" , fabs(HSPROB_B_[i][j]) , fabs(histogram.getHSPROB_B_(i , j)) , simbody);
//				fprintf(stderr , "Value of HSPROB_L_ %lf , Value of getHSPROB_L_ : %lf , value simlegs : %lf \n" , fabs(HSPROB_L_[i][j]) , fabs(histogram.getHSPROB_L_(i , j)) , simlegs);
//				ROS_INFO ("*** First Histogram *** THIS\n");
//				this->printB();
//				ROS_INFO("*** Second Histogram *** HISTO\n");
//				histogram.printB();
			}
			simplob  += sqrt( fabs(HSPROB_P_[i][j]) * fabs(histogram.getHSPROB_P_(i , j)) );
//			simhead  += sqrt( fabs(HSPROB_H_[i][j]) * fabs(histogram.getHSPROB_H_(i , j)) );
//			simbody  += sqrt( fabs(HSPROB_B_[i][j]) * fabs(histogram.getHSPROB_B_(i , j)) );
//			simlegs  += sqrt( fabs(HSPROB_L_[i][j]) * fabs(histogram.getHSPROB_L_(i , j)) );
		}
	}


//	ROS_DEBUG ("*** First Histogram ***\n");
//	this->printP();
//	ROS_DEBUG("*** Second Histogram ***\n");
//	histogram.printP();
//	histogram.printL();

	// Body and legs calc prob independientes.
	std::cerr<<std::endl;
	std::cerr<<std::endl;

/*  WITH HEAD.
	if (simplob != 0.0 && simhead != 0.0 && simbody != 0.0 && simlegs != 0.0)
		sim = simhead * simlegs * simbody;
	else if (simplob != 0.0 && simhead != 0.0 && simbody != 0.0)
		sim = simhead * simbody;
	else if (simplob != 0.0 && simbody != 0.0 && simlegs != 0.0)
		sim = simlegs * simbody;
	else
		sim = simbody;
*/
/*
	if (simhead != 0 && simbody != 0 && simlegs != 0)
		sim = simhead * simbody * simlegs;
	else if (simhead != 0 && simbody != 0)
		sim = simhead * simbody;
	else if (simbody && simlegs != 0)
		sim = simbody * simlegs;
	else if (simbody != 0)
		sim = simbody;
	else if (simlegs != 0)
		sim = simlegs;

	if (simbody > 0.9 || simlegs > 0.9 || simhead > 0.9)
		sim = 1.0;

	if (simplob > 0.8)
		sim = 1.0;
*/

	if (simplob > 0.99 && simplob < 1.01) simplob = 0.99;

//	ROS_WARN( "===================\n");
//	ROS_WARN( "Histogram Similarity Plob : %lf\n"  ,  simplob);
//	ROS_WARN( "Histogram Similarity Head : %lf\n"  ,  simhead);
//	ROS_WARN( "Histogram Similarity Body : %lf\n"  ,  simbody);
//	ROS_WARN( "Histogram Similarity Legs : %lf\n"  ,  simlegs);
//	ROS_WARN( "Histogram Similarity Total : %lf\n" ,  simplob);
//	ROS_WARN("===================\n");

//	ROS_ERROR("SLICES : \n");
//	printSlices();

	return simplob;
}

// Print a histogram.
void
HSVHistogram::printP() const
{
	fprintf(stderr , "Plob points : %d\n" , p_);
	float total = 0.0;

	for(int i=0; i<SIZEH; i++)
	{
		printf("[%lf] ", i*MAXH/SIZEH);
		for(int j=0; j<SIZES; j++)
		{
			printf("%d\t", HS_P_[i][j]);
		}
		printf("\n");
	}

	printf("%c" , '\n');
	printf("%c" , '\n');
	printf("%c" , '\n');
	printf("%c" , '\n');
	printf("%c" , '\n');

	for(int i=0; i<SIZEH; i++)
	{
		printf("[%0.6lf] ",  i*MAXH/SIZEH);
		for(int j=0; j<SIZES; j++)
		{
			printf("%lf\t", HSPROB_P_[i][j]);
			total += HSPROB_P_[i][j];
		}
		printf("\n");
	}

	fprintf(stderr , "==========================\n");
	fprintf(stderr , "Total : %lf\n" , total);
	fprintf(stderr , "==========================\n");
}


// Print a histogram.
/*
void
HSVHistogram::printH() const
{
	fprintf(stderr , "Head points : %d\n" , h_);
	float total = 0.0;

	for(int i=0; i<SIZEH; i++)
	{
		printf("[%lf] ", i*MAXH/SIZEH);
		for(int j=0; j<SIZES; j++)
		{
			printf("%d\t", HS_H_[i][j]);
		}
		printf("\n");
	}

	printf("%c" , '\n');
	printf("%c" , '\n');
	printf("%c" , '\n');
	printf("%c" , '\n');
	printf("%c" , '\n');

	for(int i=0; i<SIZEH; i++)
	{
		printf("[%0.6lf] ",  i*MAXH/SIZEH);
		for(int j=0; j<SIZES; j++)
		{
			printf("%lf\t", HSPROB_H_[i][j]);
			total += HSPROB_H_[i][j];
		}
		printf("\n");
	}

	fprintf(stderr , "==========================\n");
	fprintf(stderr , "Total : %lf\n" , total);
	fprintf(stderr , "==========================\n");
}
*/

// Print a histogram.
/*
void
HSVHistogram::printB() const
{
	fprintf(stderr , "Body points : %d\n" , b_);
	float total = 0.0;

	for(int i=0; i<SIZEH; i++)
	{
		printf("[%lf] ", i*MAXH/SIZEH);
		for(int j=0; j<SIZES; j++)
		{
			printf("%d\t", HS_B_[i][j]);
		}
		printf("\n");
	}

	printf("%c" , '\n');
	printf("%c" , '\n');
	printf("%c" , '\n');
	printf("%c" , '\n');
	printf("%c" , '\n');

	for(int i=0; i<SIZEH; i++)
	{
		printf("[%0.6lf] ",  i*MAXH/SIZEH);
		for(int j=0; j<SIZES; j++)
		{
			printf("%lf\t", HSPROB_B_[i][j]);
			total += HSPROB_B_[i][j];
		}
		printf("\n");
	}

	fprintf(stderr , "==========================\n");
	fprintf(stderr , "Total : %lf\n" , total);
	fprintf(stderr , "==========================\n");
}
*/

/*
void
HSVHistogram::printL() const
{
	fprintf(stderr , "Legs points : %d\n" , l_);
	float total = 0.0;

	for(int i=0; i<SIZEH; i++)
	{
		printf("[%lf] ", i*MAXH/SIZEH);
		for(int j=0; j<SIZES; j++)
		{
			printf("%d\t", HS_L_[i][j]);
		}
		printf("\n");
	}

	printf("%c" , '\n');
	printf("%c" , '\n');
	printf("%c" , '\n');
	printf("%c" , '\n');
	printf("%c" , '\n');

	for(int i=0; i<SIZEH; i++)
	{
		printf("[%0.6lf] ", i*MAXH/SIZEH);
		for(int j=0; j<SIZES; j++)
		{
			printf("%lf\t", HSPROB_L_[i][j]);
			total += HSPROB_L_[i][j];
		}
		printf("\n");
	}

	fprintf(stderr , "==========================\n");
	fprintf(stderr , "Total : %lf\n" , total);
	fprintf(stderr , "==========================\n");
}
*/
