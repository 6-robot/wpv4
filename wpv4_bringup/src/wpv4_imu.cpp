#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include "MPU_driver.h"

int main(int argc, char** argv)
{
    ros::init(argc,argv,"wpv4_imu");
    ROS_INFO("[wpv4_imu]");
    ros::NodeHandle n;
    ros::Publisher out_pub = n.advertise<sensor_msgs::Imu >("imu/data_raw", 1000);

    CMPU_driver m_mpu;
    ros::NodeHandle n_param("~");
    std::string strSerialPort;
    n_param.param<std::string>("serial_port", strSerialPort, "/dev/wpv4_imu");
    m_mpu.Open(strSerialPort.c_str(),1000000);
    
    ros::Rate r(100.0);

    while(n.ok())
    {
         m_mpu.ReadNewData();
         
        sensor_msgs::Imu imu_msg = sensor_msgs::Imu();	
        imu_msg.header.stamp = ros::Time::now();
        imu_msg.header.frame_id = "imu";
        imu_msg.orientation.x = m_mpu.fQuatX;
        imu_msg.orientation.y = m_mpu.fQuatY;
        imu_msg.orientation.z = m_mpu.fQuatZ;
        imu_msg.orientation.w = m_mpu.fQuatW;

        imu_msg.angular_velocity.x = m_mpu.fGyroX;
        imu_msg.angular_velocity.y = m_mpu.fGyroY;
        imu_msg.angular_velocity.z = m_mpu.fGyroZ;

        imu_msg.linear_acceleration.x = m_mpu.fAccX;
        imu_msg.linear_acceleration.y = m_mpu.fAccY;
        imu_msg.linear_acceleration.z = m_mpu.fAccZ;

        out_pub.publish(imu_msg);

        ros::spinOnce();
        r.sleep();
    }
}