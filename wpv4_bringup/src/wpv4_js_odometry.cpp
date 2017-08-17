#include <string>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <sstream>

using namespace std;

class TeleopJoy
{
public:
  TeleopJoy();
  float lx;
  float ry;
  ros::NodeHandle n;
  ros::Subscriber sub;

  //initial position
  double x;
  double y;
  double th;

  //velocity
  double vx;
  double vy;
  double vth;
  ros::Time current_time;
  ros::Time last_time;
  ros::Publisher odom_pub;
  geometry_msgs::TransformStamped odom_trans;
  tf::TransformBroadcaster broadcaster;
  nav_msgs::Odometry odom;
  geometry_msgs::Quaternion odom_quat;
private:
  void callBack(const sensor_msgs::Joy::ConstPtr& joy);
};

TeleopJoy::TeleopJoy()
{
  x = 0.0;
  y = -3.0;
  th = 0;
  vx = 1.0;
  vy = 0.0;
  vth = 0.4;
  lx = 0;
  ry = 0;
  odom_pub = n.advertise<nav_msgs::Odometry>("odom",10);
  sub = n.subscribe<sensor_msgs::Joy>("joy",10,&TeleopJoy::callBack,this);
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  //message declarations
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_footprint";
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_footprint";
  ROS_INFO("TeleopJoy");
}

void TeleopJoy::callBack(const sensor_msgs::Joy::ConstPtr& joy)
{

  ROS_INFO("Joy: [%.2f , %.2f]", lx , ry);
  lx = joy->axes[1];
  ry = joy->axes[0];

  const double degree = M_PI/180;
  current_time = ros::Time::now();
  vx = lx;
  vth = ry;

  double dt = (current_time - last_time).toSec();
  double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
  double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
  double delta_th = vth*dt;

  x += delta_x;
  y += delta_y;
  th += delta_th;

  odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0,0,th);

  //updata transform
  odom_trans.header.stamp = current_time;
  odom_trans.transform.translation.x = x;
  odom_trans.transform.translation.y = y;
  odom_trans.transform.translation.z = 0;
  odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(th);

  //filling the odometry
  odom.header.stamp = current_time;
  //position
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  //velocity
  odom.twist.twist.linear.x = vx;
  odom.twist.twist.linear.y = vy;
  odom.twist.twist.linear.z = 0;
  odom.twist.twist.angular.x = 0;
  odom.twist.twist.angular.y = 0;
  odom.twist.twist.angular.z = vth;

  last_time = current_time;

  //plublishing the odometry and new tf
  broadcaster.sendTransform(odom_trans);
  odom_pub.publish(odom);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "wpv4_js_odometry");
  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom",10);

  TeleopJoy cTeleopJoy;

  ros::spin();

  return 0;
}
