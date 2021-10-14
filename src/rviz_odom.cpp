#include <ros/ros.h>
#include <queue>
#include <cmath>
#include <mutex>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen3/Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

std::queue<nav_msgs::OdometryConstPtr> mavBuf;
std::mutex m_buf;

ros::Publisher marker_path_pub;

geometry_msgs::PoseStamped pose_stamped;
nav_msgs::Path path;

void mav_pose(const nav_msgs::OdometryConstPtr &mav_msg)
{
  m_buf.lock();
  double r = (((double) rand() / (RAND_MAX))+1)/50.0;

  pose_stamped.pose.position.x = mav_msg->pose.pose.position.x + r;
  pose_stamped.pose.position.y = mav_msg->pose.pose.position.y + r;
  pose_stamped.pose.position.z = mav_msg->pose.pose.position.z;

  pose_stamped.pose.orientation.w = mav_msg->pose.pose.orientation.w;
  pose_stamped.pose.orientation.x = mav_msg->pose.pose.orientation.x;
  pose_stamped.pose.orientation.y = mav_msg->pose.pose.orientation.y;
  pose_stamped.pose.orientation.z = mav_msg->pose.pose.orientation.z;
  pose_stamped.header.frame_id = "WORLD";
  pose_stamped.header.stamp = ros::Time::now();

  path.header.frame_id = "WORLD";
  path.header.stamp = ros::Time::now();
  path.poses.push_back(pose_stamped);
  marker_path_pub.publish(path);

  m_buf.unlock();
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "mav_path");
  ros::NodeHandle nh;
  ros::Subscriber mav_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 100, mav_pose);  
  marker_path_pub = nh.advertise<nav_msgs::Path>("mav_path", 100);


  ros::spin();
  return 0;
}
