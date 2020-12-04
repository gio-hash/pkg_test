#ifndef MY_CONTROLLER_H
#define MY_CONTROLLER_H

#include <ros/ros.h>
#include "KinematicChainControllerBase.h"
#include <boost/scoped_ptr.hpp>
#include <std_msgs/Float64MultiArray.h>


class MyController: public controller_interface::KinematicChainControllerBase<hardware_interface::EffortJointInterface>
//class MyController
{
public: 
        
        
        MyController(){}
        
        ~MyController(){}
        bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);
        void starting(const ros::Time& time);
	void update(const ros::Time& time, const ros::Duration& period);
	void command(const std_msgs::Float64MultiArray::ConstPtr &msg);
protected:

private:
        ros::NodeHandle nh_;
	/* define ROS related stuff here: subscribers, publishers, etc */
       KDL::JntArray joint_initial_states_; // joint as measured at the beginning of the control action
		KDL::JntArray current_cmd_; // command value as delta to be added to joint_initial_states_

		KDL::JntArray tau_cmd_;
		KDL::JntSpaceInertiaMatrix M_; //Inertia matrix
		KDL::JntArray C_, G_;	//Coriolis and Gravitational matrices
		KDL::JntArray Kp_, Kv_;	//Position and Velocity gains

		boost::scoped_ptr<KDL::ChainDynParam> id_solver_;

};

#endif
