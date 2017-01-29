/*
 * Arms.h
 *
 *  Created on: Jun 2, 2016
 *      Author: javi
 */

#ifndef INCLUDE_PEOPLE_TRACKER_BODY_H_
#define INCLUDE_PEOPLE_TRACKER_BODY_H_

#include "people_tracker/Pblob.h"

class Body : public Pblob {

	public:

		Body(const Pblob& plob);
		virtual ~Body();

		/*
		 * Test arms inside the
		 * siluete.
		 */
		void setArms();
		bool areArms() const;

		/*
		 * Info about class arms.
		 */
		std::list<pcl::PointXYZRGB> getArms () const { return arms_; };
		void print () const;

	private :

		bool isSiteArmsZ (const pcl::PointXYZRGB& point) const;
		bool isSiteArmsY (const pcl::PointXYZRGB& point) const;
		bool isSiteArms  (const pcl::PointXYZRGB& point) const;

		bool isLineArms (const pcl::PointXYZRGB& point , float delta);
		void getLineArms (std::list<pcl::PointXYZRGB>& line , float delta);
		bool checkArms(const std::list<pcl::PointXYZRGB>& line);
		void removeArms(std::list<pcl::PointXYZRGB>& line);
		void print (const std::list<pcl::PointXYZRGB>& line);

		/*
		 * Min dimensions one line to consider part of
		 * body with arms  in the siluete.
		 */
		static const int MINPOINTSTOAMRS = 50;
		/*
		 * The distance between points in the line
		 * of the siluete at most twice separation
		 * points.
		 */
		static const int MAXPOINTSEPARATESBODY = 2;

		/*
		 * Minimal lines to consider the separation
		 * between body and arms like arms separate
		 * of body.
		 */
		static const int MINIMALLINESTOARMS = 9;
		static const float DISTANCEBETWEENBODYANDARM = 0.15;


		/*
		 * Cut the siluete to get the arms of the siluete.
		 */
 		static const float HEIGHDIVIDE 	 = 4.0;
		static const float WIDTHDIVIDE 	 = 6.0;

		/*
		 * Const of body in one siluete, is possible
		 * bad idea if the siluete is to long from
		 * camera it can not coincide.
		 */
		static const float SIZEZARMSMIN  = 0.6;
		static const float SIZEZARMSMAX  = 1.6;

		/*
		 * Scan of body and arms in search of
		 * separation between body and arms.
		 *
		 * Is good idea scan all the body
		 * to fill out the histogram of
		 * up part of siluete.
		 */
		static const float INITARMSDELTA = 0.3;
		static const float INCREASE 	 = 0.008;
		static const float SCANZARMS = 0.5;

		std::list<pcl::PointXYZRGB> arms_;
		std::list<pcl::PointXYZRGB> body_;

		HSVHistogram histogram_;

		float sAx_ , sAy_ , sAz_;
		int numDistArms_;
};

#endif /* INCLUDE_PEOPLE_TRACKER_BODY_H_ */
