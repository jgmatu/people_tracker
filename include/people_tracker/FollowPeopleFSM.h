#ifndef FOLLOWPEOPLEFSM_H_
#define FOLLOWPEOPLEFSM_H_

#include <bica_core/Component.h>
#include <ros/ros.h>

#include <std_msgs/String.h>

namespace bica {

class FollowPeopleFSM: public bica::Component
{
public:
	FollowPeopleFSM();
	virtual ~FollowPeopleFSM();

	void activateCode();

	virtual void Following_code_iterative() {};
	virtual void Following_code_once() {};
	virtual void Lost_code_iterative() {};
	virtual void Lost_code_once() {};
	virtual void Initial_code_iterative() {};
	virtual void Initial_code_once() {};
	virtual void Finish_code_iterative() {};
	virtual void Finish_code_once() {};
	virtual void Training_code_iterative() {};
	virtual void Training_code_once() {};
	virtual void StopTraining_code_iterative() {};
	virtual void StopTraining_code_once() {};
	virtual void Returning_code_iterative() {};
	virtual void Returning_code_once() {};

	virtual bool Following_2_Lost() {return false;};
	virtual bool Initial_2_Training() {return false;};
	virtual bool Following_2_Returning() {return false;};
	virtual bool Training_2_StopTraining() {return false;};
	virtual bool StopTraining_2_Training() {return false;};
	virtual bool Lost_2_Following() {return false;};
	virtual bool Returning_2_Finish() {return false;};
	virtual bool StopTraining_2_Following() {return false;};


	bool ok();

protected:
	ros::Time state_ts_;

private:

	void step() {};

	void deactivateAllDeps();
	void Following_activateDeps();
	void Lost_activateDeps();
	void Initial_activateDeps();
	void Finish_activateDeps();
	void Training_activateDeps();
	void StopTraining_activateDeps();
	void Returning_activateDeps();


	static const int FOLLOWING = 0;
	static const int LOST = 1;
	static const int INITIAL = 2;
	static const int FINISH = 3;
	static const int TRAINING = 4;
	static const int STOPTRAINING = 5;
	static const int RETURNING = 6;
	

	int state_;

	std::string myBaseId_;
	ros::NodeHandle nh_;
	ros::Publisher state_pub_;


};

} /* namespace bica */

#endif /* FOLLOWPEOPLEFSM_H_ */
