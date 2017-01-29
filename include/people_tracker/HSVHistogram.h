/*
 * HSVHistogram.h
 *
 *  Created on: 28/04/2016
 *      Author: Paco
 */

#ifndef HSVHISTOGRAM_H_
#define HSVHISTOGRAM_H_

#include "ros/ros.h"

#include <list>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>

#define MAXH 360.0
#define MAXS 1.0
#define MAXV 1.0
#define MINVALIDV 0.1
#define MAXVALIDV 0.9
#define MINVALIDS 0.1

class HSVHistogram {
public:

	HSVHistogram();
	HSVHistogram(const HSVHistogram& other);
	virtual ~HSVHistogram();


	void add(pcl::PointXYZRGB& pointcolor);
	void add(pcl::PointXYZHSV& pointcolor);
	void add(const HSVHistogram& histogram);

//	bool inRange(pcl::PointXYZRGB& pointcolor) const;
//	bool inRange(pcl::PointXYZHSV& pointcolor) const;

	/*
	 * Return the probability the histogram you pass and the histogram
	 * actual are the same histogram.
	 */
	float similarity(const HSVHistogram& histogram) const;


	/*
	 * Number of points that represent a hue and saturation
	 * range.
	 */
	int getHS_P_(int idH, int idS) const 		{ return HS_P_[idH][idS];	  };
	float getHSPROB_P_(int idH , int idS) const { return HSPROB_P_[idH][idS]; };

//	int getHS_H_(int idH, int idS) const 		{ return HS_H_[idH][idS];	  };
//	float getHSPROB_H_(int idH , int idS) const { return HSPROB_H_[idH][idS]; };

//	int getHS_B_(int idH, int idS) const 		{ return HS_B_[idH][idS];	  };
//	float getHSPROB_B_(int idH , int idS) const { return HSPROB_B_[idH][idS]; };

//	int getHS_L_(int idH, int idS) const 		{ return HS_L_[idH][idS];	  };
//	float getHSPROB_L_(int idH , int idS) const { return HSPROB_L_[idH][idS]; };


	/*
	 * Copy the histogram in a new place in memory and
	 * return a pointer to the new histogram the old
	 * histogram is not touched.
	 */
	HSVHistogram& operator=(const HSVHistogram& other);


	// This method plus point to point two histograms.
	friend HSVHistogram operator+(HSVHistogram lhs, HSVHistogram rhs)
	{
		lhs.p_ += rhs.p_;
//		lhs.h_ += rhs.h_;
//		lhs.b_ += rhs.b_;
//		lhs.l_ += rhs.l_;

		for(int i = 0; i < HSVHistogram::SIZEH; i++) {
			for(int j = 0; j < HSVHistogram::SIZES; j++) {
				lhs.HS_P_[i][j] += rhs.getHS_P_(i , j);
//				lhs.HS_H_[i][j] += rhs.getHS_H_(i , j);
//				lhs.HS_B_[i][j] += rhs.getHS_B_(i , j);
//				lhs.HS_L_[i][j] += rhs.getHS_L_(i , j);
 			}
		}

		lhs.setProbs();
		return lhs;
	}


	// This method multiply the histogram to one scalar.. A = a*A
	friend HSVHistogram operator*(HSVHistogram lhs , float factor)
	{

		for(int i = 0; i < HSVHistogram::SIZEH; i++) {
			for(int j = 0; j < HSVHistogram::SIZES; j++) {
				lhs.HS_P_[i][j] *= factor;
//				lhs.HS_H_[i][j] *= factor;
//				lhs.HS_B_[i][j] *= factor;
//				lhs.HS_L_[i][j] *= factor;
			}
		}

		lhs.setProbs();
		return lhs;
	}


	void printP() const;
	void printH() const;
	void printB() const;
	void printL() const;
	void printSlices() const;


	int getP() const { return p_; };
//	int getH() const { return h_; };
//	int getB() const { return b_; };
//	int getL() const { return l_; };

	const std::list<pcl::PointXYZRGB>& getDep() const { return dep_; };
	const int getNDep() const { return static_cast<int>(dep_.size());};

private:

	void inizialize();

	bool isHead (const pcl::PointXYZHSV& pointcolor) const;
	bool isBody (const pcl::PointXYZHSV& pointcolor) const;
	bool isLegs (const pcl::PointXYZHSV& pointcolor) const;

	void setPositionXYZ (const pcl::PointXYZRGB& pointSource , pcl::PointXYZHSV& pointDest);
	void setPositionXYZ (const pcl::PointXYZHSV& pointSource , pcl::PointXYZRGB& pointDest);

	void setProbs();


	void depHist (pcl::PointXYZHSV pointcolor);

/*
  				KOBUKI
	static const float MAXHEAD = 1.20;
	static const float MINHEAD = 0.95;

	static const float MAXBODY = 0.92;
	static const float MINBODY = 0.63;

	static const float MAXLEGS = 0.5;
	static const float MINLEGS = 0.0;

	static const int SIZEH = 60;
	static const int SIZES = 30;
*/
/*
	static const float MAXHEAD = 2.1;
	static const float MINHEAD = 1.95;
*/

//	static const float MAXBODY = -0.09;
//	static const float MINBODY = -0.10;

/*
	static const float MAXLEGS = 1.0;
	static const float MINLEGS = 0.0;
*/

//	static const int SIZEH = 30;			Kobuki Real.
//	static const int SIZES = 10;

	static const int SIZEH = 20;
	static const int SIZES = 10;

	int p_ /*, h_ , b_ , l_*/;

	int HS_P_[SIZEH][SIZES] /*, HS_H_[SIZEH][SIZES] , HS_B_[SIZEH][SIZES] , HS_L_[SIZEH][SIZES] */;
	float HSPROB_P_[SIZEH][SIZES] /*, HSPROB_H_[SIZEH][SIZES] , HSPROB_B_[SIZEH][SIZES] , HSPROB_L_[SIZEH][SIZES]*/;

	std::list<pcl::PointXYZRGB> dep_;
};

#endif /* HSVHISTOGRAM_H_ */
