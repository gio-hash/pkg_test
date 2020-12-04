#include "ros/ros.h"
#include "std_msgs/String.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include <sstream>
#include "KinematicChainControllerBase.h"
#include <boost/scoped_ptr.hpp>
/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::Publisher joint_trajecctory_pub = n.advertise<trajectory_msgs::JointTrajectory>("/lwr/joint_trajectory_controller/command", 1000);
  ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  
  trajectory_msgs::JointTrajectory msg_trajectory;
msg_trajectory.header.stamp = ros::Time::now();
  msg_trajectory.joint_names.resize(7);
  msg_trajectory.joint_names[0] = "lwr_a1_joint";

  msg_trajectory.joint_names[1] = "lwr_a2_joint";
  msg_trajectory.joint_names[2] = "lwr_e1_joint";

  msg_trajectory.joint_names[3] = "lwr_a3_joint";
  

  msg_trajectory.joint_names[4] = "lwr_a4_joint";

  msg_trajectory.joint_names[5] = "lwr_a5_joint";

  msg_trajectory.joint_names[6] = "lwr_a6_joint";

  

  
  
  trajectory_msgs::JointTrajectoryPoint points_n;
  points_n.positions.resize(1);
  points_n.positions.push_back(2);
  msg_trajectory.points.push_back(points_n); 
  joint_trajecctory_pub.publish(msg_trajectory);
  //msg_trajectory.points[0].positions[0] = 2;
  //msg_trajectory.points[0].positions.push_back(2);
  //msg_trajectory.points[0].positions[0]=2;
  boost::scoped_ptr<KDL::ChainDynParam> id_solver_;
  while (ros::ok())
  {
      msg_trajectory.header.stamp = ros::Time::now();
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);
    msg_trajectory.points[0].time_from_start = ros::Duration(1);
    joint_trajecctory_pub.publish(msg_trajectory);
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}