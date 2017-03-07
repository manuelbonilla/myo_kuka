
#include <ros/ros.h>
#include <ros/console.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Float64.h>

float data;

void callback(const std_msgs::Float64& msg)
{
  data = msg.data;
  // ROS_INFO_STREAM("msg received: " <<  data);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "HandRepublisher");
  ros::NodeHandle node;
  ROS_INFO("[HandRepublisher] Node is ready");
  std::string on_topic, synergy_joint, from_topic;
  node.param <std::string>("from_topic", from_topic, "hand_position");
  node.param <std::string>("on_topic", on_topic, "right_hand/joint_trajectory_controller/command");
  node.param <std::string>("synergy_joint", synergy_joint, "left_hand_synergy_joint");
  

  ROS_INFO_STREAM("from topic: " << from_topic.c_str());
  ROS_INFO_STREAM("on topic: " << on_topic.c_str());
  ROS_INFO_STREAM("synergy_joint: " << synergy_joint.c_str());

  double spin_rate = 10;

  ros::Rate rate(spin_rate);
  ros::Publisher hand_publisher = node.advertise<trajectory_msgs::JointTrajectory>(on_topic.c_str(), 1000);
  ros::Subscriber hand_subscriber = node.subscribe(from_topic.c_str(), 1, callback);

  data = 0;

  while (node.ok())
  {
    trajectory_msgs::JointTrajectory msg_jointT_hand;
    msg_jointT_hand.header.stamp = ros::Time::now();
    msg_jointT_hand.points.resize(1);
    msg_jointT_hand.joint_names.resize(1);
    msg_jointT_hand.points[0].positions.resize(1);
    msg_jointT_hand.points[0].positions[0] = data;
    msg_jointT_hand.points[0].time_from_start = ros::Duration(0.1); // 1s;
    msg_jointT_hand.joint_names[0] = synergy_joint.c_str();
    hand_publisher.publish(msg_jointT_hand);
    ros::spinOnce();
    rate.sleep();

  }
  return 0;
}


