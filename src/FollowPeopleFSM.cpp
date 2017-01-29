
#include <people_tracker/FollowPeopleFSM.h>

namespace bica {

FollowPeopleFSM::FollowPeopleFSM()
: state_(INITIAL),
  myBaseId_("FollowPeopleFSM")
{
	state_ts_ = ros::Time::now();
	state_pub_ = nh_.advertise<std_msgs::String>("/"+myBaseId_+"/state", 1, false);

}

FollowPeopleFSM::~FollowPeopleFSM()
{
      ;
}

void
FollowPeopleFSM::activateCode()
{
	deactivateAllDeps();

	state_ = INITIAL;
	state_ts_ = ros::Time::now();

	Initial_activateDeps();
	Initial_code_once();

}


bool
FollowPeopleFSM::ok()
{
	if(active_)
	{
		std_msgs::String msg;

		switch(state_) {
	      case FOLLOWING:

	           Following_code_iterative();

	                msg.data = "Following";
	if(Following_2_Lost())
	{

	deactivateAllDeps();

	state_ = LOST;
	state_ts_ = ros::Time::now();

	Lost_activateDeps();
	Lost_code_once();
	}
	if(Following_2_Returning())
	{

	deactivateAllDeps();

	state_ = RETURNING;
	state_ts_ = ros::Time::now();

	Returning_activateDeps();
	Returning_code_once();
	}
	state_pub_.publish(msg);
	break;

	case LOST:

	Lost_code_iterative();

	msg.data = "Lost";
	if(Lost_2_Following())
	{

	deactivateAllDeps();

	state_ = FOLLOWING;
	state_ts_ = ros::Time::now();

	Following_activateDeps();
	Following_code_once();
	}
	state_pub_.publish(msg);
	break;

	case INITIAL:

	Initial_code_iterative();

	msg.data = "Initial";
	if(Initial_2_Training())
	{

	deactivateAllDeps();

	state_ = TRAINING;
	state_ts_ = ros::Time::now();

	Training_activateDeps();
	Training_code_once();
	}
	state_pub_.publish(msg);
	break;

	case FINISH:

	Finish_code_iterative();

	msg.data = "Finish";
	state_pub_.publish(msg);
	break;

	case TRAINING:

	Training_code_iterative();

	msg.data = "Training";
	if(Training_2_StopTraining())
	{

	deactivateAllDeps();

	state_ = STOPTRAINING;
	state_ts_ = ros::Time::now();

	StopTraining_activateDeps();
	StopTraining_code_once();
	}
	state_pub_.publish(msg);
	break;

	case STOPTRAINING:

	StopTraining_code_iterative();

	msg.data = "StopTraining";
	if(StopTraining_2_Training())
	{

	deactivateAllDeps();

	state_ = TRAINING;
	state_ts_ = ros::Time::now();

	Training_activateDeps();
	Training_code_once();
	}

	if(StopTraining_2_Following())
	{

	deactivateAllDeps();

	state_ = FOLLOWING;
	state_ts_ = ros::Time::now();

	Following_activateDeps();
	Following_code_once();
	}
	state_pub_.publish(msg);
	break;

	case RETURNING:

	Returning_code_iterative();

	msg.data = "Returning";
	if(Returning_2_Finish())
	{

	deactivateAllDeps();

	state_ = FINISH;
	state_ts_ = ros::Time::now();

	Finish_activateDeps();
	Finish_code_once();
	}
	state_pub_.publish(msg);
	break;


		}
	}

	return Component::ok();
}

void
FollowPeopleFSM::deactivateAllDeps()
{
	removeDependency("people_node");
	removeDependency("navigator");
};

void
FollowPeopleFSM::Following_activateDeps()
{
	addDependency("people_node");
	addDependency("navigator");
}

void
FollowPeopleFSM::Lost_activateDeps()
{
	addDependency("people_node");
}

void
FollowPeopleFSM::Initial_activateDeps()
{
}

void
FollowPeopleFSM::Finish_activateDeps()
{
}

void
FollowPeopleFSM::Training_activateDeps()
{
	addDependency("people_node");
}

void
FollowPeopleFSM::StopTraining_activateDeps()
{
	addDependency("people_node");
}

void
FollowPeopleFSM::Returning_activateDeps()
{

}




} /* namespace bica */
