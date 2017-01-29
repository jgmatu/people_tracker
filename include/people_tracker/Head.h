/*
 * Head.h
 *
 *  Created on: May 30, 2016
 *      Author: javi
 */

#ifndef INCLUDE_PEOPLE_TRACKER_HEAD_H_
#define INCLUDE_PEOPLE_TRACKER_HEAD_H_

#include "people_tracker/Pblob.h"

#define FLOAT_MIN  -340282346638528859811704183484516925440.0

class Head : public Pblob {

	public:

		Head(const Pblob& pblob);
		virtual ~Head();

		bool isHead();

		/*
		 * Info from class head.
		 */
		std::list<pcl::PointXYZRGB> getHead () const { return head_; };

	private :


		static const float MAXPOINTSHEAD = 500;

		// Dimesions of one head like max.
		static const float MAXSIZE = 0.30;
		static const float MINSIZE = 0.10; // Kobuki real.

		static const float BORDER = 0.04;
		static const float BODY = 0.20;

		std::list<pcl::PointXYZRGB> head_;

		bool isPointLimitZ (pcl::PointXYZRGB point);
		void print (float sHy , float sy , int nhead) const;

};



#endif /* INCLUDE_PEOPLE_TRACKER_HEAD_H_ */
