/*
 * Legs.h
 *
 *  Created on: May 31, 2016
 *      Author: javi
 */

#ifndef INCLUDE_PEOPLE_TRACKER_LEGS_H_
#define INCLUDE_PEOPLE_TRACKER_LEGS_H_

#include "people_tracker/Pblob.h"

class Legs : public Pblob {

	public :

		Legs ();
		Legs (const Pblob& pblob);
		virtual ~Legs();

		/*
		 * Test legs inside the siluete.
		 */
		void setLegs ();
		bool hasLegs ();
		float similarity(const Legs& other) const {return histogram_.similarity(other.getHistogram());};


		void operator=(const Legs& other);

		/*
		 * Info about class legs.
		 */;
		std::list<pcl::PointXYZRGB> getLegs() const { return legs_; };
		void print () const;

	private :

		/*
		 * Cut the siluete in parts evaluating
		 * its dimensions...
		 */
		static const int WIDTHDIVIDE = 3;
		static const int HEIGHDIVIDE = 2;

		/*
		 * Dimensions necessary to consider the
		 * separation between points like a
		 * human legs.
		 */
		static const int MINPOINTSLINE = 60;


		/*
		 * Only one points or a minimal number of points are
		 * considered like legs in separation between points
		 * more points could be consider an error measured.
		 */
		static const int MAXPOINTSSEPARATESLINE = 2;

		/*
		 * Minimal lines of siluete to consider the siluete
		 * have a distance of legs.
		 */
		static const int MINLINELEGSCONSIDERLEGS = 24;

		/*
		 * Legs start in 1.0 metter to down.
		 */
		static const float SITELEGS = 1.0;

		/*
		 * Min points distance to cosider legs.
		 */
		static const float MINLEGSDISTANCE = 0.10;

		/*
		 * Increase must be a low number to detect well the
		 * distance between point in axis y.
		 */
		static const float INCREASE = 0.008;
		/*
		 * ScanZ Is the values of Z we detected from lower
		 * siluete to up and detect separation between points
		 * 0.3 is 30 cm from minimal down to up.
		 */
		static const float SCANZSUPERFICE = 0.55;

		std::list<pcl::PointXYZRGB> legs_;

		float sLx_ , sLy_ , sLz_;
		int  nlegs_ , legsCount_;

		bool isSizeLegsDimensions() const;

		bool isPointLimitYLegs(pcl::PointXYZRGB point);
		bool isPointLimitZLegs(pcl::PointXYZRGB point);
		bool isLine (pcl::PointXYZRGB point , float delta);
		void removeLegs (std::list<pcl::PointXYZRGB>& line);
		bool isLegs(pcl::PointXYZRGB point , float delta);

		bool areLegs (pcl::PointXYZRGB pointA , pcl::PointXYZRGB pointB);
		bool isSiteLegs(pcl::PointXYZRGB pointA);

		void getLineLegs(float delta , std::list <pcl::PointXYZRGB> & line);
		bool checkLegLine(const std::list<pcl::PointXYZRGB>& line);

		bool isDistance (pcl::PointXYZRGB pointA , pcl::PointXYZRGB pointB);

		void print (const std::list<pcl::PointXYZRGB>& line);
};

#endif /* INCLUDE_PEOPLE_TRACKER_LEGS_H_ */
