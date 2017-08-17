#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>


void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    //ROS_INFO("Imu Seq: [%d]", msg->header.seq);
    //ROS_INFO("Imu Orientation x: [%f], y: [%f], z: [%f], w: [%f]", msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);

    static tf::TransformBroadcaster odom_broadcaster;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(0, 0, 0) );
    tf::Quaternion q(msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
    printf("[imu_odometry] Yaw= %f \n",tf::getYaw(q)*180/3.1415);
    transform.setRotation(q);
    odom_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_footprint"));
    odom_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "imu"));
}

int main(int argc, char **argv)
{
  ros::init(argc,argv, "imu_odometry"); 
  ROS_INFO("[imu_odometry]");

  ros::NodeHandle n;
  //odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  ros::Subscriber sub = n.subscribe("imu/data_raw", 1000, imuCallback);
  ros::spin();

  return 0;
}