#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include "WPV4_driver.h"
#include <math.h>

/*!******************************************************************
 底盘类型选择
 ********************************************************************/
#define CT_4WD        1   //四轮驱动底盘模式
#define CT_TRACK      2   //三轮全向底盘模式
#define CT_MECANUM    3   //麦克纳姆轮全向底盘模式

static int nChassisType = CT_4WD;

static CWPV4_driver m_wpv4;
static int nLastMotorPos[4];
static geometry_msgs::Twist lastVel;
void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    //ROS_INFO("[wpv4_cmd_vel] liner(%.2f %.2f) angular(%.2f)", msg->linear.x,msg->linear.y,msg->angular.z);
    switch(nChassisType)
    {
        case CT_4WD:
            m_wpv4.Velocity(msg->linear.x,0,msg->angular.z);
            break;
        case CT_TRACK:
            m_wpv4.Track(msg->linear.x,0,msg->angular.z);
            break;
        case CT_MECANUM:
            m_wpv4.Mecanum(msg->linear.x,msg->linear.y,msg->angular.z);
            break;
    }
    

    lastVel.linear.x = msg->linear.x*2.9;
    lastVel.linear.y = 0;
    lastVel.angular.z = msg->angular.z*2.85;
}

static bool bFirstYaw = true;
static double fYawZero = 0;
static double fCurYaw = 0;
void IMUCallback(const sensor_msgs::Imu::ConstPtr & imu)
{
     tf::Quaternion quat(imu->orientation.x, imu->orientation.y, imu->orientation.z, imu->orientation.w);
     double yaw = 0;//tf::getYaw(quat);
     double roll = 0;
     double pitch = 0;
     tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
     //ROS_WARN("[IMU] yaw= %.2f, roll= %.2f, pitch= %.2f",yaw, roll, pitch);
     if(bFirstYaw == false)
     {
        fCurYaw = yaw;
     }
     else
     {
         //第一祯
         fYawZero = yaw;
         fCurYaw = fYawZero;
         bFirstYaw = false;
     }
}
 
static float fKVx = 1.0f/sqrt(3.0f);
static float fKVy = 2.0f/3.0f;
static float fKVz = 1.0f/3.0f;
static float fSumX =0;
static float fSumY =0;
static float fSumZ =0;
static float fOdomX =0;
static float fOdomY =0;
static float fOdomZ =0;
static geometry_msgs::Pose2D lastPose;
int main(int argc, char** argv)
{
    ros::init(argc,argv,"wpv4_core");
    ros::NodeHandle n;
    ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel",1,&cmdVelCallback);
    ros::Subscriber sub_imu = n.subscribe("/imu/data_raw", 10, IMUCallback);

    ros::NodeHandle n_param("~");
    std::string strSerialPort;
    n_param.param<std::string>("serial_port", strSerialPort, "/dev/wpv4_base");
    m_wpv4.Open(strSerialPort.c_str(),115200);
    bool bOdom = true;
    n_param.param<bool>("odom", bOdom, true);
    bool bImuOdom = false;
    n_param.param<bool>("imu_odom", bImuOdom, false);
    bool bJointStatePub = true;
    n_param.param<bool>("joint_state", bJointStatePub, false);
    std::string chassis_type;
    if (n_param.getParam("chassis_type", chassis_type))
    {
        printf("[设置]底盘类型= %s \n",chassis_type.c_str());
        if(chassis_type == "4wd")
        {
            printf("[设置]四轮驱动底盘 \n");
            nChassisType = CT_4WD;
        }
        if(chassis_type == "track")
        {
            printf("[设置]履带底盘 \n");
            nChassisType = CT_TRACK;
        }
        if(chassis_type == "mecanum")
        {
            printf("[设置]麦克纳姆轮底盘 \n");
            nChassisType = CT_MECANUM;
        }
    }
    else
    {
        printf("[设置]未设置底盘类型,默认为四轮驱动底盘 \n");
        nChassisType = CT_4WD;
    }
    
    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    ros::Rate r(30.0);

    ros::Publisher joint_state_pub = n.advertise<sensor_msgs::JointState>("/joint_states",100);
    sensor_msgs::JointState msg;
    std::vector<std::string> joint_name(6);
    std::vector<double> joint_pos(6);

    joint_name[0] = "base_to_lf_wheal";
    joint_name[1] = "base_to_rf_wheal";
    joint_name[2] = "base_to_lb_wheal";
    joint_name[3] = "base_to_rb_wheal";
    joint_name[4] = "kinect_height";
    joint_name[5] = "kinect_pitch";
    joint_pos[0] = 0.0f;
    joint_pos[1] = 0.0f;
    joint_pos[2] = 0.0f;
    joint_pos[3] = 0.0f;
    joint_pos[4] = 0.50f;    //kinect_height
    joint_pos[5] = -0.17f;    //kinect_pitch
    
    ros::Publisher odom_pub;
    geometry_msgs::TransformStamped odom_trans;
    tf::TransformBroadcaster broadcaster;
    nav_msgs::Odometry odom;
    geometry_msgs::Quaternion odom_quat;

    odom_pub = n.advertise<nav_msgs::Odometry>("odom",5);
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";
    odom_trans.transform.translation.x = 0;
    odom_trans.transform.translation.y = 0;
    odom_trans.transform.translation.z = 0;
    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0);
    odom.pose.pose.position.x = 0;
    odom.pose.pose.position.y = 0;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    odom.twist.twist.linear.x = 0;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.linear.z = 0;
    odom.twist.twist.angular.x = 0;
    odom.twist.twist.angular.y = 0;
    odom.twist.twist.angular.z = 0;

    lastPose.x = lastPose.y = lastPose.theta = 0;
    lastVel.linear.x = lastVel.linear.y = lastVel.linear.z = lastVel.angular.x = lastVel.angular.y = lastVel.angular.z = 0;
    nLastMotorPos[0] = nLastMotorPos[1] = nLastMotorPos[2] = nLastMotorPos[3] =0;

    while(n.ok())
    {
        //ROS_INFO("[] m_wpv4.ReadNewData");
        m_wpv4.ReadNewData();
        m_wpv4.nParseCount ++;

         //ROS_INFO("[m_wpv4.nParseCount]= %d",m_wpb_home.nParseCount);
        if(m_wpv4.nParseCount > 100)
        {
            m_wpv4.arMotorPos[0] =0; nLastMotorPos[0] = 0;
            m_wpv4.arMotorPos[1] =0; nLastMotorPos[1] = 0;
            m_wpv4.arMotorPos[2] =0; nLastMotorPos[2] = 0;
            m_wpv4.arMotorPos[3] =0; nLastMotorPos[3] = 0;
            m_wpv4.nParseCount = 0;
            //ROS_INFO("empty");
        }
        
        last_time = current_time;
        current_time = ros::Time::now();

        if(bOdom == true)
        {
            ///////////
            double fVx,fVy,fVz;
            double fPosDiff[4];
            if(nLastMotorPos[0] != m_wpv4.arMotorPos[0] || nLastMotorPos[1] != m_wpv4.arMotorPos[1] || nLastMotorPos[2] != m_wpv4.arMotorPos[2] || nLastMotorPos[3] != m_wpv4.arMotorPos[3])
            {
                double fTimeDur = current_time.toSec() - last_time.toSec();
                fPosDiff[0] = (double)(m_wpv4.arMotorPos[0] - nLastMotorPos[0]); 
                fPosDiff[1] = (double)(m_wpv4.arMotorPos[1] - nLastMotorPos[1]);
                // fPosDiff[2] = (double)(m_wpv4.arMotorPos[2] - nLastMotorPos[2]);
                fVx = (fPosDiff[1] - fPosDiff[0]) * 0.5;
                fVz = (fPosDiff[0] + fPosDiff[1]) * 0.5;
                //////////////////////////////////
                //方案一 通过电机码盘结算里程计数据
                // fVx = fVx/(fTimeDur*45400);
                // fVy = 0;
                // fVz = fVz*3.14159*2/(fTimeDur*103400);
                //////////////////////////////////
                // 方案二 直接把下发速度当作里程计积分依据
                fVx = lastVel.linear.x;
                fVy = lastVel.linear.y;
                fVz = lastVel.angular.z;
               
                ///////////////////////////////////
                //ROS_INFO("[odom] liner(%.2f %.2f) angular(%.2f)",fVx,fVy,fVz);
                double dx = (lastVel.linear.x*cos(lastPose.theta) - lastVel.linear.y*sin(lastPose.theta))*fTimeDur;
                double dy = (lastVel.linear.x*sin(lastPose.theta) + lastVel.linear.y*cos(lastPose.theta))*fTimeDur;

                //lastPose中就是积分出来的里程计数据
                lastPose.x += dx;
                lastPose.y += dy;
                if(bImuOdom == false)
                {
                    lastPose.theta += (fVz*fTimeDur);
                }
                else
                {
                    //用陀螺
                    ROS_INFO("[odom_imu]");
                    lastPose.theta = (fCurYaw - fYawZero);
                }
                //lastPose.theta = (double)((nLastMotorPos[0] + nLastMotorPos[1] + nLastMotorPos[2])*fKVz)/1925;
                ROS_INFO("[odom] x=%.2f y=%.2f a=%.2f",lastPose.x,lastPose.y,lastPose.theta);
                //ROS_INFO("[pos] x=%.2f linera( %.2f , %.2f ) -%.2f",lastPose.x, lastVel.linear.x,lastVel.linear.y,lastPose.theta);

                //ROS_INFO("px=%.2f a=%.2f (lx=%.2f cos=%.2f - ly=%.2f sin=%.2f)",lastPose.x,lastPose.theta, lastVel.linear.x,cos(lastPose.theta),lastVel.linear.y,sin(lastPose.theta));

                odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0,0,lastPose.theta);
                //updata transform
                odom_trans.header.stamp = current_time;
                odom_trans.transform.translation.x = lastPose.x;
                odom_trans.transform.translation.y = lastPose.y;
                odom_trans.transform.translation.z = 0;
                odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(lastPose.theta);

                //filling the odometry
                odom.header.stamp = current_time;
                //position
                odom.pose.pose.position.x = lastPose.x;
                odom.pose.pose.position.y = lastPose.y;
                odom.pose.pose.position.z = 0.0;
                odom.pose.pose.orientation = odom_quat;

                //velocity
                odom.twist.twist.linear.x = fVx;
                odom.twist.twist.linear.y = fVy;
                odom.twist.twist.linear.z = 0;
                odom.twist.twist.angular.x = 0;
                odom.twist.twist.angular.y = 0;
                odom.twist.twist.angular.z = fVz;

                //plublishing the odometry and new tf
                broadcaster.sendTransform(odom_trans);
                odom_pub.publish(odom);

                // lastVel.linear.x = fVx;
                // lastVel.linear.y = fVy;
                // lastVel.angular.z = fVz;

                nLastMotorPos[0] = m_wpv4.arMotorPos[0];
                nLastMotorPos[1] = m_wpv4.arMotorPos[1];
                nLastMotorPos[2] = m_wpv4.arMotorPos[2];
                nLastMotorPos[3] = m_wpv4.arMotorPos[3];

            }
            else
            {
                odom_trans.header.stamp = ros::Time::now();
                //plublishing the odometry and new tf
                broadcaster.sendTransform(odom_trans);
                odom_pub.publish(odom);
                //ROS_INFO("[odom] zero");
            }
            ///////////
        }

        if(bJointStatePub == true)
        {
            msg.header.stamp = ros::Time::now();
            msg.header.seq ++;
            msg.name = joint_name;
            msg.position = joint_pos;
            joint_state_pub.publish(msg);
        }

        ros::spinOnce();
        r.sleep();
    }
}