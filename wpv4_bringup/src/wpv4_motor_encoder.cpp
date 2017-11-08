#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "WPV4_driver.h"

static CWPV4_driver m_wpv4;
void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    //ROS_INFO("[wpv4_cmd_vel] liner(%.2f %.2f) angular(%.2f)", msg->linear.x,msg->linear.y,msg->angular.z);
    m_wpv4.Velocity(msg->linear.x,msg->linear.y,msg->angular.z);
}

static int nFirstVal[4];
static bool bFirst = true;
int main(int argc, char** argv)
{
    ros::init(argc,argv,"wpv4_motor_encoder");
    ros::NodeHandle n;
    ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel",1,&cmdVelCallback);

    ros::NodeHandle n_param("~");
    std::string strSerialPort;
    n_param.param<std::string>("serial_port", strSerialPort, "/dev/ttyUSB0");
    m_wpv4.Open(strSerialPort.c_str(),115200);
    
    ros::Rate r(100.0);
    r.sleep();

    while(n.ok())
    {
        m_wpv4.ReadNewData();

        ///////////////////
        ROS_INFO("enc [M1]%d [M2]%d [M3]%d [M4]%d", m_wpv4.arMotorPos[0], m_wpv4.arMotorPos[1], m_wpv4.arMotorPos[2], m_wpv4.arMotorPos[3]);
        ////////////////////
        if(bFirst == true)
        {
            nFirstVal[0] = m_wpv4.arMotorPos[0];
            nFirstVal[1] = m_wpv4.arMotorPos[1];
            nFirstVal[2] = m_wpv4.arMotorPos[2];
            nFirstVal[3] = m_wpv4.arMotorPos[3];
            bFirst = false;
        }
        else
        {
            int nDiff[4];
            nDiff[0] = m_wpv4.arMotorPos[0] - nFirstVal[0];
            nDiff[1] = m_wpv4.arMotorPos[1] - nFirstVal[1];
            nDiff[2] = m_wpv4.arMotorPos[2] - nFirstVal[2];
            nDiff[3] = m_wpv4.arMotorPos[2] - nFirstVal[3];
            //ROS_INFO("Encoder [M1]%d [M2]%d [M3]%d", m_wpv4.arMotorPos[0], m_wpv4.arMotorPos[1], m_wpv4.arMotorPos[2]);
            //ROS_INFO("Diff [M1]%d [M2]%d [M3]%d", nDiff[0], nDiff[1], nDiff[2]);
        }
        /////////////////////
        
        ros::spinOnce();
        r.sleep();
    }
}