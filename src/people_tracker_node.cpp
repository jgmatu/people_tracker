#include <ros/ros.h>
#include <ros/console.h>

#include "people_tracker/PlobPerception.h"
#include "people_tracker/PlobTracker.h"
#include "geometry_msgs/Twist.h"

#include <tf/transform_broadcaster.h>

#include <math.h>

#include <std_srvs/Trigger.h>
#include <visualization_msgs/MarkerArray.h>

#include <bica_core/Component.h>


bool
byHistogram (const PlobTracker& track1  , const  PlobTracker& track2)
{
	return track1.getHistogram().similarity(track2.getHistogram()) > 0.8;
}


class PeopleNode : public bica::Component
{

public:

	PeopleNode() :
		counter(-1),
		idtracker(-1),
		state_(INITIAL),
		plobPerception(),
		trackeds(),
		countTrain(0)
	{
		start_srv_ = nh_.advertiseService(ros::this_node::getName()+"/start_training", &PeopleNode::startTrainingCb, this);
		stop_srv_ = nh_.advertiseService(ros::this_node::getName()+"/stop_training", &PeopleNode::stopTrainingCb, this);
		trainer_zone = nh_.advertise<visualization_msgs::MarkerArray>( "/visualization_markerZoneTrainer", 10);
	}

	~PeopleNode()
	{
		;
	}

	void
	publish_zoneTrainer()
	{
		if(trainer_zone.getNumSubscribers() == 0) return;

		visualization_msgs::MarkerArray markers;

		visualization_msgs::Marker marker;
		marker.header.frame_id = "base_footprint";
		marker.header.stamp = ros::Time();
		marker.ns = "trainer_zone";
		marker.id = counter;
		marker.type = visualization_msgs::Marker::CUBE;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = XTRAINER_POSE;
		marker.pose.position.y = YTRAINER_POSE;
		marker.pose.position.z = ZTRAINER_POSE;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		marker.scale.x = XTRAINER_SIZE;
		marker.scale.y = YTRAINER_SIZE;
		marker.scale.z = ZTRAINER_SIZE;
		marker.color.a = 0.5; // Don't forget to set the alpha! why? ROS documentation about visual markers visibility of object... :).
		marker.color.r = 0.0;
		marker.color.g = 1.0;
		marker.color.b = 0.0;

		markers.markers.push_back(marker);

		trainer_zone.publish(markers);
	}



	bool
	startTrainingCb(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp)
	{
		state_= TRAINING;
		ROS_INFO("RECEIVED!!!");
		return true;
	}

	bool
	stopTrainingCb(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp)
	{

		if (countTrain > 2)
		{
			resp.success = true;
			resp.message = "Ok!";
			state_= READY;
		}
		else
		{
			resp.success = false;
			resp.message = "Fail!";
			state_ = INITIAL;
		}
		return true;
	}


	bool
	isTracker(PlobTracker tracker)
	{
		return tracker.getId() == idtracker;
	}

	bool
	oldTracker (PlobTracker tracker)
	{
		return (ros::Time::now() - tracker.getLastCorrected()).toSec() > 5.0;
	}

	void
	removeTracks(std::list<PlobTracker>& trackeds)
	{
		bool deleteTracker = false;

		std::list<PlobTracker>::iterator iti= trackeds.begin();

		while(iti != trackeds.end())
		{
			if(oldTracker(*iti) && !isTracker(*iti))
			{
//				fprintf(stderr, "removeTracks : Removed track. \n");
				if (!deleteTracker && iti->getId() == idtracker)
				{
					deleteTracker = true;
					idtracker = -1;
				}
				iti -> removeMarkers();
				iti = trackeds.erase(iti);
			}
			else
			{
				if (deleteTracker)
				{
					idtracker = iti->getId();
					deleteTracker = false;
				}
				++iti;
			}
		}

		drop_sim_tracker();

		if (idtracker < 0)
		{
//			ROS_ERROR("removeTrakcs : Not trackers found!!! \n");
			idtracker = -1;
		}
	}

	std::list<PlobTracker>::iterator
	getTrackToApply(std::list<PlobTracker>& trackeds, const std::vector<Pblob>::const_iterator& perception)
	{
		float max = THRESHOLD;
		std::list<PlobTracker>::iterator ret = trackeds.end();
		std::list<PlobTracker>::iterator it  = trackeds.begin();

	//	std::cerr<<std::endl;
	//	ROS_INFO("=====================================================\n");
	//	ROS_INFO("NUMBER OF trackeds : %d \n" , (int)trackeds.size());
	//	ROS_INFO("ONE PERCEPTION..... \n");

		trackeds.sort(byHistogram);

		while(it != trackeds.end())
		{
			float sim = it->getSimilarity(perception);

//			fprintf(stderr, "getTrackToApply : [%lf] \n", sim);
			if(sim > max)
			{
				max = sim;
				ret = it;
			}
			++it;
		}

		if(ret == trackeds.end())
		{
//			fprintf(stderr, "\tNo perception suitable\n");
		}
		else
		{
//			fprintf(stderr, "\tSim = %lf\n", max);
		}
		std::cerr<<std::endl;
//		ROS_INFO("=====================================================\n");
		return ret;
	}

	void
	createTrack(std::list<PlobTracker>& trackeds, const std::vector<Pblob>::const_iterator& perception, int counter)
	{
		trackeds.push_back(PlobTracker(counter, perception->getX(), perception->getY(), perception->getHistogram() , perception->getZmax()));

	//	ROS_INFO("Tracked add to the list of trackers... \n");
	//	ROS_INFO("Position : (%lf , %lf) \n" , perception->getX() , perception->getY());
	//	ROS_INFO("Counter : %d \n" , counter);
	}

	void
	add (std::vector<Pblob>& permanents , const Pblob& permanent)
	{
		permanents.push_back(permanent);
	}

	void
	send_tf_pose_tracker(const PlobTracker& tracker)
	{
	  static tf::TransformBroadcaster br;

	  tf::Transform transform;
	  tf::Quaternion q;

	  if (std::isnan(tracker.getX()) || std::isnan(tracker.getY()))
		  ROS_ERROR("NAN IN POSE TRACKER SPACEVELVECTOR!!!!!!");

	  transform.setOrigin(tf::Vector3(tracker.getX(), tracker.getY() , 0.0));

	  q.setRPY(0, 0, atan2(tracker.getY() , tracker.getX()));
	  transform.setRotation(q);
	  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/base_footprint", "/people_track"));
	}


	void
	publish_trackers ()
	{
		for(std::list<PlobTracker>::const_iterator it = trackeds.begin(); it != trackeds.end(); ++it)
		{
//			ROS_WARN("Tracker Id : %d \n" , it->getId());
			it->publishMarkers(idtracker);
		}
	}

	void
	drop_sim_tracker()
	{
		std::list<PlobTracker>::iterator it = trackeds.begin();

		std::list<PlobTracker>::iterator track_follow;
		bool found = false;
		while (it != trackeds.end()) {

			if (it->getId() == idtracker)
			{
				track_follow = it;
				found = true;
			}
			else if (found && track_follow->getSimTrack(it) > THRESHOLD)
			{
				it = trackeds.erase(it);
			}

			it++;
		}
	}

	void
	ready (std::vector<Pblob>& perception)
	{
		// Predict.
		for(std::list<PlobTracker>::iterator it = trackeds.begin(); it != trackeds.end(); ++it)
			it->predict();


//		fprintf(stderr, "\nC Perceptions: %zu\tTracks: %zu\n", perception.size(), trackeds.size());
		if(!perception.empty())
		{
//			fprintf(stderr, "====================================================================\n");
//			fprintf(stderr, "Processing perceptions\n");

			// int count = counter;
			std::vector<Pblob>::iterator itp = perception.begin();
			while(itp != perception.end())
			{
//				fprintf(stderr, "E Perceptions: %zu\tTracks: %zu\n", perception.size(), trackeds.size());

				std::list <PlobTracker> ::iterator selfTrack;
				selfTrack = getTrackToApply(trackeds, itp);
//				fprintf(stderr, "F Perceptions: %zu\tTracks: %zu\n", perception.size(), trackeds.size());

				if(selfTrack != trackeds.end())
				{
//					fprintf(stderr, "Correcting track perceptions...");
					selfTrack->correct(*itp);
//					fprintf(stderr, "done\n");
				}
				else
				{
//					fprintf(stderr, "Correcting new track...");
					createTrack(trackeds, itp, counter++);
//					fprintf(stderr, "done\n");
				}
//				fprintf(stderr, "[%d / %zu]\n", counter, perception.size());

				itp = perception.erase(itp);
			}
//			fprintf(stderr, "Removing unused tracks...\n");
			removeTracks(trackeds);
		}

		publish_trackers();

		for (std::list<PlobTracker>::const_iterator it = trackeds.begin() ; it != trackeds.end() ; it++) {
			if (it->getId() == idtracker) {
				if ((ros::Time::now() - it->getLastCorrected()).toSec() < TIME_OUT) {
					send_tf_pose_tracker(*it);
				}
			}
		}
	}

	bool
	isZoneX (const std::vector<Pblob>::const_iterator& perception)
	{
		ROS_WARN("Perception ->Limit X Left : %lf , Limit X Rigth : %lf \n" , perception->getX() + perception->getXmin() , perception->getX() + perception->getXmax());
		ROS_WARN("Zone ->Limit X Left : %lf , Limit X Rigth : %lf \n" ,XTRAINER_POSE - XTRAINER_SIZE , XTRAINER_POSE + XTRAINER_SIZE);

		return perception->getX() + perception->getXmin() > XTRAINER_POSE - XTRAINER_SIZE && perception->getX() + perception->getXmax() < XTRAINER_POSE + XTRAINER_SIZE;
	}

	bool
	isZoneY (const std::vector<Pblob>::const_iterator& perception)
	{
		ROS_WARN("Zone ->Limit Y Left : %lf , Limit Y Rigth : %lf \n" ,YTRAINER_POSE - YTRAINER_SIZE, YTRAINER_POSE + YTRAINER_SIZE);
		ROS_WARN("Perception ->Limit Y Left : %lf , Limit Y Rigth : %lf \n" , perception->getY() + perception->getYmin() , perception->getY() + perception->getYmax());

		return perception->getY() + perception->getYmin() > YTRAINER_POSE - YTRAINER_SIZE && perception->getY() + perception->getYmax() < YTRAINER_POSE + YTRAINER_SIZE;
	}


	bool
	isZoneTrainer(const std::vector<Pblob>::const_iterator& perception)
	{
		return isZoneX(perception) && isZoneY(perception);
	}

	void
	training(const std::vector<Pblob>& perceptions)
	{

		if(trackeds.empty() && !perceptions.empty() && isZoneTrainer(perceptions.begin()))
		{
			fprintf(stderr, "Created first track\n");
			idtracker = 0;
			counter = idtracker;
			createTrack(trackeds, perceptions.begin(), counter++);
			// Print signature first tracker...
			// perception.back().getHistogram().print();
			countTrain++;
		}

		if (!perceptions.empty() && isZoneTrainer(perceptions.begin()) && !trackeds.empty() && trackeds.begin()->getSimilarity(perceptions.begin()) > THRESHOLD)
		{
			countTrain++;
			ROS_ERROR("Training tracker... %d \n" , countTrain);
		}
		else
		{
			countTrain = 0;
			trackeds.clear();
		}

		if (!perceptions.empty() && isZoneTrainer(perceptions.begin())) ROS_ERROR("IS INSIDE ZONE TRAINER!!!!!!");

		publish_zoneTrainer();

		if (!trackeds.empty()) publish_trackers();
	}


	void
	step()
	{
		if(!isActive()) return;

//		fprintf(stderr, "=========================================================\n");
		std::vector<Pblob> perception = plobPerception.getPerception();

		switch (state_)
		{

		case INITIAL :

			ROS_INFO("INITAL!!");
			countTrain = 0;
			break;

		case TRAINING :

			ROS_INFO("TRAINING!!");
			training(perception);
			break;

		case READY:

			ROS_INFO("READY!!");
			ready(perception);
			break;

		case LOST :

			break;

		default :

			ROS_ERROR("FAIL!!!!! : %d \n" , state_);

		}

		plobPerception.clear();

//		ROS_INFO("[%s] step", ros::this_node::getName().c_str());
	}

	private:

		static const int LOST = 0;
		static const int INITIAL = 1;
		static const int READY = 2;
		static const int TRAINING = 3;


		static const float XTRAINER_POSE = 3.5;
		static const float YTRAINER_POSE = 0.0;
		static const float ZTRAINER_POSE = 0.0;

		static const float XTRAINER_SIZE = 3.0;
		static const float YTRAINER_SIZE = 1.2;
		static const float ZTRAINER_SIZE = 0.5;

		static const float TIME_OUT = 3.0;


		PlobPerception plobPerception;
		std::list<PlobTracker> trackeds;
		int counter , idtracker , state_;
		int countTrain;


		ros::NodeHandle nh_;

		ros::ServiceServer stop_srv_;
		ros::ServiceServer start_srv_;
		ros::Publisher trainer_zone;

};

int
main(int argc, char** argv)
{
	ros::init(argc, argv, "people_node");
	PeopleNode people_node;


	ros::Rate loop_rate(10);
	while(people_node.ok())
	{
		people_node.step();

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
