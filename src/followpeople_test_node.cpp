
#include <ros/ros.h>
#include <ros/console.h>

#include <tf/transform_listener.h>

#include <people_tracker/FollowPeopleFSM.h>

#include <geometry_msgs/Pose2D.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose2D.h>

#define LOST_PEOPLE_TIME 7.0 // Medir donde queda el tracker despues del tiempo de espera por lost..
#define FOUND_PEOPLE_TIME 2.0
#define INIT_TIME 5.0
#define TIME_TRAIN 6.0

class FollowPeopleFSMImpl: public bica::FollowPeopleFSM
{

public:

	FollowPeopleFSMImpl() :
		scan_topic_("/attention/scan"),
		target_frame_topic_("/attention/target_frame"),
		train_sucess_(false),
		train_sent_(false),
		voice_train_(true),
		navigator_topic_("/navigator/pose2D"),
		lastGoalx_(0),
		lastGoaly_(0),
		init_x_(0),
		init_y_(0),
		init_a_(0),
		countGoals(0),
		finished_(false)
	{
		command_sub_ =  nh_.subscribe<std_msgs::String>("/human_command", 1, &FollowPeopleFSMImpl::commandCB, this);
		attention_scan_pub_ = nh_.advertise<std_msgs::String>(scan_topic_, 1);
		attention_tf_pub_ = nh_.advertise<std_msgs::String>(target_frame_topic_, 1);
		p_voice_command_ = nh_.advertise<std_msgs::String>("/wm_voice_component/voice_component_input",1000);
		navigator_pub_ = nh_.advertise<geometry_msgs::Pose2D>(navigator_topic_, 1);
		finish_sub_ = nh_.subscribe<std_msgs::Bool>("/navigator/finished", 1, &FollowPeopleFSMImpl::goalReachedCB, this);
		vel_pub_  = nh_.advertise<geometry_msgs::Twist>("/rb1_base_controller_v0/cmd_vel", 1);

		nh_.param("/dn_amcl/initial_pose_x", init_x_, init_x_);
		nh_.param("/dn_amcl/initial_pose_y", init_y_, init_y_);
		nh_.param("/dn_amcl/initial_pose_a", init_a_, init_a_);

		start_sc_ =  nh_.serviceClient<std_srvs::Empty>("/people_node/start_training"); // <------------ check
		stop_sc_ =  nh_.serviceClient<std_srvs::Trigger>("/people_node/stop_training"); // <------------ check

		startTime_ = ros::Time::now();

		ROS_INFO("INSTANCED!!!!! \n");
	}

	void send_voice (std::string voice) {
        std_msgs::String msg;
        std::stringstream ss;

        ss << voice << std::endl;
        msg.data = ss.str();
        p_voice_command_.publish(msg);


		ROS_WARN("SEND VOICE COMMAND!!!");
	}

	~FollowPeopleFSMImpl() {
		;
	}

	void Initial_code_once() {
		ROS_INFO("READY TO START!!!");
		startTime_ = ros::Time::now();
	}

	void Training_code_once() {
		std_srvs::Empty srv;

		ROS_INFO("TRAINING START!!");
		startTime_ = ros::Time::now();

		send_voice("we are looking for you");

		if (voice_train_)
			send_voice("go backward");
		else
			send_voice("go forward");

		voice_train_ = !voice_train_;
		start_sc_.call(srv);
	}

	void StopTraining_code_once() {
		std_srvs::Trigger srv;
		stop_sc_.call(srv);

		train_sucess_ = srv.response.success;
		train_sent_ = true;

		ROS_INFO("TRAINING STOPED!!!");
	}


	bool listmap2people() {
		try {
			tfListener_.waitForTransform("/map", "/people_track" , ros::Time(0), ros::Duration(0.5));
			tfListener_.lookupTransform("/map" , "/people_track", ros::Time(0), map2people_);// <---- check
		} catch (tf::TransformException& ex) {
			ROS_ERROR("%s",ex.what());
			return false;
		}
		return true;
	}

	void Following_code_iterative() {
		std_msgs::String attention_cmdmsg;

		attention_cmdmsg.data = "/people_track";
		attention_tf_pub_.publish(attention_cmdmsg);

		ROS_INFO("FOLLOWING!!!!");
		if (!listmap2people())
			return;

		tf::StampedTransform map2bf;
		try {
			tfListener_.waitForTransform("/map", "/base_footprint" , ros::Time(0), ros::Duration(0.5));
			tfListener_.lookupTransform("/map", "/base_footprint", ros::Time(0), map2bf);
		} catch (tf::TransformException& ex) {
			ROS_ERROR("%s",ex.what());
			return;
		}

		double x, y, a;
		double vx, vy, m;

		vx = map2people_.getOrigin().getX() - map2bf.getOrigin().getX();
		vy = map2people_.getOrigin().getY() - map2bf.getOrigin().getY();

		m = sqrt(vx*vx + vy*vy);
		vx = vx / m;
		vy = vy / m;

		x =  map2people_.getOrigin().getX() - vx;
		y =  map2people_.getOrigin().getY() - vy;

		a = atan2(map2people_.getOrigin().getY()-y, map2people_.getOrigin().getX()-x);

		geometry_msgs::Pose2D nav_pose;
		nav_pose.x = x;
		nav_pose.y = y;
		nav_pose.theta = a;

		float dist_x = lastGoalx_ - x;
		float dist_y = lastGoaly_ - y;
		float dist = sqrt (dist_x * dist_x + dist_y * dist_y);

		ROS_INFO("DIST LAST GOAL TO GOAL : %lf" , dist);
		if (countGoals == 0 || dist > DISTANCE) {
			lastGoalx_ = x;
			lastGoaly_ = y;
			countGoals++;
			navigator_pub_.publish(nav_pose);
			ROS_INFO("people (%lf, %lf) bf: (%lf, %lf) goal: (%lf, %lf)",
					map2people_.getOrigin().getX(),
					map2people_.getOrigin().getY(),
					map2bf.getOrigin().getX(),
					map2bf.getOrigin().getY(),
					x, y);
		}
	}

	void Lost_code_iterative() {
		std_msgs::String attention_cmdmsg;
		attention_cmdmsg.data = "";
		attention_scan_pub_.publish(attention_cmdmsg);

		if (!listmap2people())
			return;

		geometry_msgs::Twist cmd;
		cmd.angular.z = 0.05;
		vel_pub_.publish(cmd);
	}

	void Returning_code_once() {
		geometry_msgs::Pose2D nav_pose;

		nav_pose.x = init_x_;
		nav_pose.y = init_y_;
		nav_pose.theta = init_a_;

		navigator_pub_.publish(nav_pose);
	}

	void Finish_code_once() {
		ROS_INFO("FINISHED");
		send_voice("Final");
	}

	bool Initial_2_Training() {
		ROS_INFO("[%s]", last_command_.c_str());
		ROS_INFO("[%s]" , ( last_command_ == "CONTINUE" ) ? "true" : "false" );
		if(last_command_ == "CONTINUE") {
			last_command_ = "";
			return true;
		} else {
			return false;
		}
	}

	bool Following_2_Lost() {
		if ((ros::Time::now() - map2people_.stamp_).toSec() > LOST_PEOPLE_TIME) {
			send_voice("I lost you");
			return true;
		} else {
			return false;
		}
	}

	bool StopTraining_2_Following() {
		if (train_sent_ && train_sucess_) {
			send_voice("Ready to follow");
			return true;
		} else {
			return false;
		}
	}

	bool Training_2_StopTraining() {
		return (ros::Time::now() - startTime_).toSec() > TIME_TRAIN;
	}

	bool Lost_2_Following() {
		if((ros::Time::now() - map2people_.stamp_).toSec() < FOUND_PEOPLE_TIME) {
			geometry_msgs::Twist cmd;
			cmd.angular.z = 0.00;
			vel_pub_.publish(cmd);
			send_voice("I found you");
			return true;
		} else {
			return false;
		}
	}

	bool StopTraining_2_Training() {
		if(train_sent_ && !train_sucess_)
		{
			train_sent_ = false;
			return true;
		}
		else
			return false;
	}

	bool Following_2_Returning() {
		if(last_command_ == "CONTINUE") {
			last_command_ = "";
			return true;
		} else
			return false;
	}

	bool Returning_2_Finish() {
		return finished_;
	}

private:

	void commandCB(const std_msgs::String::ConstPtr& msg) {
		last_command_ = msg->data;
	}

	void goalReachedCB(const std_msgs::Bool::ConstPtr& msg) {
		finished_ = msg->data;
	}

	static const float DISTANCE = 0.70;

	ros::NodeHandle nh_;

	ros::Publisher attention_scan_pub_;
	ros::Publisher attention_tf_pub_;
	ros::Publisher p_voice_command_;
	ros::Publisher navigator_pub_;
	ros::Subscriber command_sub_;
	ros::Subscriber finish_sub_;
	ros::Publisher vel_pub_;

	bool finished_;

	move_base_msgs::MoveBaseGoal goal_pose_;
	std::string last_command_;


	tf::TransformListener tfListener_;

	tf::StampedTransform map2people_;

	std::string scan_topic_;
	std::string target_frame_topic_;
	std::string navigator_topic_;

	ros::ServiceClient start_sc_, stop_sc_;

	ros::Time startTime_;

	bool train_sucess_, train_sent_, voice_train_;

	float lastGoalx_ , lastGoaly_;
	int countGoals;

	float init_x_ , init_y_ , init_a_;
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "people_test");

	FollowPeopleFSMImpl followpeople_test;

	ros::Rate loop_rate(5);
	while(followpeople_test.ok()) {
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
