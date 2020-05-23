#include <waterplus_local_planner/wpv4_diff_local_planner.h>
#include <tf_conversions/tf_eigen.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>

// register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS( waterplus_local_planner::WPV4DiffLocalPlanner, nav_core::BaseLocalPlanner)

static float target_vel_x = 0;
static float target_vel_z = 0;
static float ac_vel_x = 0;
static float ac_vel_z = 0;

namespace waterplus_local_planner
{
    static double fScaleD2R = 3.14159*0.5 / 180;
    //激光雷达回调(避障)
    void WPV4DiffLocalPlanner::lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
    {
        return;
        //int nRanges = scan->ranges.size();
        //ROS_WARN("[WLP]range=%d ang= %.2f ",nRanges,scan->angle_increment); 

        int nRanges = scan->ranges.size();
        float range_max = scan->range_max;
        float range_min = scan->range_min;
        const float ac_width = 0.30;
        const float ac_dist = 0.5;

        float left_obst = 1.0;      //记录障碍物在左方的的距离
        float right_obst = 1.0;     //记录障碍物在右方的的距离
        float front_obst = ac_dist+1; //记录障碍物在正前方的的距离

        //right front
        for(int i=180;i<360;i++)
        {
             if(scan->ranges[i] < range_min || scan->ranges[i] > range_max)
                continue;
            float dist = scan->ranges[i] * sin((i-180)*fScaleD2R);
            if(dist < ac_dist)
            {
                float width = scan->ranges[i] * cos((i-180)*fScaleD2R);
                if(width < right_obst)
                {
                    right_obst = width;
                }
                if(dist < front_obst)
                {
                    front_obst = dist;
                }
            }
        }

        //left front
        for(int i=360;i<540;i++)
        {
             if(scan->ranges[i] < range_min || scan->ranges[i] > range_max)
                continue;
            float dist = scan->ranges[i] * sin((540-i)*fScaleD2R);
            if(dist < ac_dist)
            {
                float width = scan->ranges[i] * cos((540-i)*fScaleD2R);
                if(width < left_obst)
                {
                    left_obst = width;
                }
                 if(dist < front_obst)
                {
                    front_obst = dist;
                }
            }
        }

        // ajust
        if(right_obst > ac_width && left_obst > ac_width)
        {
            m_bAC = false;
        }
        else
        {
            m_bAC = true;
            if(right_obst < left_obst)
            {
                //m_nAC_action = AC_TURN_LEFT;
                ac_vel_z = 0.3;
            }
            else
            {
                //m_nAC_action = AC_TURN_RIGHT;
                ac_vel_z = -0.3;
            }
            ac_vel_x = front_obst - ac_dist;
        }
    }

    //构造函数
    WPV4DiffLocalPlanner::WPV4DiffLocalPlanner()
    {
        //ROS_WARN("[WLP]WPV4DiffLocalPlanner() "); 
        m_costmap_ros = NULL;
        m_tf_listener = NULL; 
        m_goal_reached = false;
        m_bInitialized = false;
        m_bAC = false;
        m_bFirstStep = true;
    }

    WPV4DiffLocalPlanner::~WPV4DiffLocalPlanner()
    {
    }

    void WPV4DiffLocalPlanner::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros)
    {
        ROS_WARN("WPV4DiffLocalPlanner::initialize() ");
        if(!m_bInitialized)
        {	
            m_tf_listener = tf;
            m_costmap_ros = costmap_ros;
            m_costmap = m_costmap_ros->getCostmap();
            
            m_global_frame_id = m_costmap_ros->getGlobalFrameID();      //"odom"
            m_robot_base_frame_id = m_costmap_ros->getBaseFrameID();    //"base_footprint"
            
            m_footprint_spec = m_costmap_ros->getRobotFootprint();
            costmap_2d::calculateMinAndMaxDistances(m_footprint_spec, m_robot_inscribed_radius, m_robot_circumscribed_radius); 

            ros::NodeHandle nh_planner("~/" + name);
            nh_planner.param("max_vel_trans", m_max_vel_trans, 1.0);
            nh_planner.param("max_vel_rot", m_max_vel_rot, 1.0);
            nh_planner.param("max_acc_trans", m_max_acc_trans, 0.5);
            nh_planner.param("max_acc_rot", m_max_acc_rot, 0.5);
            nh_planner.param("acc_scale_trans", m_acc_scale_trans, 0.5);
            nh_planner.param("acc_scale_rot", m_acc_scale_rot, 0.6);
            nh_planner.param("path_dist_tolerance", m_path_dist_tolerance, 0.5);
            nh_planner.param("goal_dist_tolerance", m_goal_dist_tolerance, 0.2);
            nh_planner.param("goal_yaw_tolerance", m_goal_yaw_tolerance, 0.2);

            ROS_WARN("max_vel_trans = %.2f ",m_max_vel_trans);
            ROS_WARN("acc_scale_trans = %.2f ",m_acc_scale_trans);

            m_pub_target = nh_planner.advertise<geometry_msgs::PoseStamped>("local_planner_target", 10);
            m_scan_sub = nh_planner.subscribe<sensor_msgs::LaserScan>("/scan",1,&WPV4DiffLocalPlanner::lidarCallback,this);
            
            m_bInitialized = true;

            ROS_DEBUG("wpv4_diff_local_planner plugin initialized.");
        }
        else
        {
            ROS_WARN("wpv4_diff_local_planner has already been initialized, doing nothing.");
        }
    }

    void WPV4DiffLocalPlanner::initialize(std::string name,tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
    {
        ROS_WARN("WPV4DiffLocalPlanner::initialize() ");
        if(!m_bInitialized)
        {	
            m_tf_listener = new tf::TransformListener;
            m_costmap_ros = costmap_ros;
            m_costmap = m_costmap_ros->getCostmap();
            
            m_global_frame_id = m_costmap_ros->getGlobalFrameID();      //"odom"
            m_robot_base_frame_id = m_costmap_ros->getBaseFrameID();    //"base_footprint"
            
            m_footprint_spec = m_costmap_ros->getRobotFootprint();
            costmap_2d::calculateMinAndMaxDistances(m_footprint_spec, m_robot_inscribed_radius, m_robot_circumscribed_radius); 

            ros::NodeHandle nh_planner("~/" + name);
            nh_planner.param("max_vel_trans", m_max_vel_trans, 1.0);
            nh_planner.param("max_vel_rot", m_max_vel_rot, 1.0);
            nh_planner.param("max_acc_trans", m_max_acc_trans, 0.5);
            nh_planner.param("max_acc_rot", m_max_acc_rot, 0.5);
            nh_planner.param("acc_scale_trans", m_acc_scale_trans, 0.5);
            nh_planner.param("acc_scale_rot", m_acc_scale_rot, 0.6);
            nh_planner.param("path_dist_tolerance", m_path_dist_tolerance, 0.5);
            nh_planner.param("goal_dist_tolerance", m_goal_dist_tolerance, 0.2);
            nh_planner.param("goal_yaw_tolerance", m_goal_yaw_tolerance, 0.2);

            ROS_WARN("max_vel_trans = %.2f ",m_max_vel_trans);
            ROS_WARN("acc_scale_trans = %.2f ",m_acc_scale_trans);

            m_pub_target = nh_planner.advertise<geometry_msgs::PoseStamped>("local_planner_target", 10);
            m_scan_sub = nh_planner.subscribe<sensor_msgs::LaserScan>("/scan",1,&WPV4DiffLocalPlanner::lidarCallback,this);
            
            m_bInitialized = true;

            ROS_DEBUG("wpv4_diff_local_planner plugin initialized.");
        }
        else
        {
            ROS_WARN("wpv4_diff_local_planner has already been initialized, doing nothing.");
        }
    }

    bool WPV4DiffLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
    {
        // ROS_WARN("[WLP]setPlan() ");
        if(!m_bInitialized)
        {
            ROS_ERROR("wpv4_diff_local_planner has not been initialized, please call initialize() before using this planner");
            return false;
        }

        m_global_plan.clear();
        m_global_plan = plan;
        m_nPathIndex = 0;

        m_goal_reached = false;
        m_nStep = WLP_STEP_GOTO;
        
        return true;
    }

    static double CalDirectAngle(double inFromX, double inFromY, double inToX, double inToY)
    {
        double res = 0;
        double dx = inFromX - inToX;
        double dy = -(inFromY - inToY);
        if (dx == 0)
        {
            if (dy > 0)
            {
                res = 180 - 90;
            }
            else
            {
                res = 0 - 90;
            }

        }
        else
        {
            double fTan = dy / dx;
            res = atan(fTan) * 180 / 3.1415926;

            if (dx < 0)
            {
                res = res - 180;
            }

        }
        res = 180 - res;

        if (res < 0)
        {
            res += 360;
        }

        if (res > 360)
        {
            res -= 360;
        }

        res = res*3.1415926/180;

        return res;
    }

    static double AngleFix(double inAngle, double inMin, double inMax)
    {
        if (inMax - inMin > 6.28)
        {
            return inAngle;
        }
        
        double retAngle = inAngle;
        while (retAngle < inMin)
        {
            retAngle += 6.28;
        }
        while (retAngle > inMax)
        {
            retAngle -= 6.28;
        }
        return retAngle;
    }

    bool WPV4DiffLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
    {
        // ROS_WARN("[WLP]computeVelocityCommands() ");
        if(!m_bInitialized)
        {
            ROS_ERROR("wpv4_diff_local_planner has not been initialized, please call initialize() before using this planner");
            return false;
        }

        int path_num = m_global_plan.size();
        if(path_num == 0)
        {
            return false;
        }

        cmd_vel.linear.x = 0;
        cmd_vel.linear.y = 0;
        cmd_vel.angular.z = 0;
        
        if(m_nStep == WLP_STEP_ARRIVED)
        {
            ROS_WARN("[WLP_ARRIVED](%.2f %.2f):%.2f",cmd_vel.linear.x,cmd_vel.linear.y,cmd_vel.angular.z);
            return true;
        }

         /////////////////////////////////////////////////////////////
        double goal_x,goal_y,goal_th;
        getTransformedPosition(m_global_plan.back(), m_robot_base_frame_id, goal_x, goal_y, goal_th);
        //ROS_WARN("goal(%.2f dy= %.2f) th= %.2f",goal_x, goal_y, goal_th);

        //  double face_goal = CalDirectAngle(0, 0, goal_x, goal_y);
        //  face_goal = AngleFix(face_goal,-2.1,2.1);
        //  ROS_WARN("face = %.2f goal(%.2f dy= %.2f) th= %.2f",face_goal, goal_x, goal_y, goal_th);
        
        if(m_nStep == WLP_STEP_GOTO)
        {
            // check if global goal is near
            double goal_dist = sqrt(goal_x*goal_x + goal_y*goal_y);
            if(goal_dist < m_goal_dist_tolerance)
            {
                m_nStep = WLP_STEP_NEAR;
                ROS_WARN("[WLP-GOTO] -> [WLP_NEAR] (%.2f,%.2f) %.2f",goal_x, goal_y, goal_th);
            }
            else
            {
                //check if path is near
                double path_x, path_y, path_th;
                while(m_nPathIndex < path_num-1)
                {
                    getTransformedPosition(m_global_plan[m_nPathIndex], m_robot_base_frame_id, path_x, path_y, path_th);
                    if(sqrt(path_x*path_x + path_y*path_y) < m_path_dist_tolerance)
                    {
                        m_nPathIndex ++;
                        ROS_WARN("[WLP-GOTO]path index = %d ",m_nPathIndex);
                    }
                    else
                    {
                        break;  //target is far enough
                    }
                }

                double face_path = CalDirectAngle(0, 0, path_x, path_y);
                face_path = AngleFix(face_path,-2.1,2.1);
                if(fabs(face_path)> 1.0)//0.82)
                {
                    //与目标航点角度太大,原地转
                    //turn in place
                    target_vel_x = 0;
                    target_vel_z = face_path * m_acc_scale_rot * 1.5;
                    if(target_vel_z > 0) target_vel_z +=1.5;
                    if(target_vel_z < 0) target_vel_z -=1.5;
                }
                else
                {
                    //正常行驶,计算目标速度
                    double path_dist = sqrt(path_x*path_x + path_y*path_y);
                    target_vel_x = path_dist*cos(face_path) * m_acc_scale_trans;
                    //target_vel_z = path_dist*sin(face_path) * m_acc_scale_trans;
                    target_vel_z = face_path * m_acc_scale_rot;
                    //ROS_WARN("cmd_vel.linear.x = %.2f ",cmd_vel.linear.x);
                    if(target_vel_x > 0) target_vel_x+=0;
                    if(target_vel_x < 0) target_vel_x-=0;
                    if(target_vel_z > 0) target_vel_z+=0;
                    if(target_vel_z < 0) target_vel_z-=0;
                    // cmd_vel.linear.x = 0;
                    // cmd_vel.linear.y = 0;
                    //ROS_WARN("++ cmd_vel.linear.x = %.2f ",cmd_vel.linear.x);
                }
                m_pub_target.publish(m_global_plan[m_nPathIndex]);
            }
        }

        if(m_nStep == WLP_STEP_NEAR)
        {
            //到达目标点附近,调整朝向即可
            target_vel_x = 0;
            target_vel_z = goal_th;

            if(fabs(goal_th) < m_goal_yaw_tolerance)
            {
                m_goal_reached = true;
                m_nStep = WLP_STEP_ARRIVED;
                target_vel_z = 0;
                ROS_WARN("[WLP-ARRIVED] goal (%.2f,%.2f) %.2f",goal_x, goal_y, goal_th);

            }

            m_pub_target.publish(m_global_plan.back());
        }

        cmd_vel = m_last_cmd;

        //////////////////
        //避障速度叠加
        cmd_vel.linear.x = ac_vel_x;
        cmd_vel.angular.z = ac_vel_z;
        //////////////////

        //前后移动加速度控制
        if( fabs(target_vel_x - cmd_vel.linear.x) >  m_max_acc_trans)
        {
            //超过加速度极限,减小幅度
            if(target_vel_x > cmd_vel.linear.x)
            {
                cmd_vel.linear.x += m_max_acc_trans;
            }
            else
            {
                cmd_vel.linear.x -= m_max_acc_trans;
            }
        }
        else
        {
            cmd_vel.linear.x = target_vel_x;
        }

        //旋转加速度控制
        if( fabs(target_vel_z - cmd_vel.angular.z) >  m_max_acc_rot)
        {
            //超过加速度极限,减小幅度
            if(target_vel_z > cmd_vel.angular.z)
            {
                cmd_vel.angular.z += m_max_acc_rot;
            }
            else
            {
                cmd_vel.angular.z -= m_max_acc_rot;
            }
        }
        else
        {
            cmd_vel.angular.z = target_vel_z;
        }

        //速度极限控制
        if(cmd_vel.linear.x > m_max_vel_trans) cmd_vel.linear.x = m_max_vel_trans;
        if(cmd_vel.linear.x < -m_max_vel_trans) cmd_vel.linear.x = -m_max_vel_trans;
        if(cmd_vel.linear.y > m_max_vel_trans) cmd_vel.linear.y = m_max_vel_trans;
        if(cmd_vel.linear.y < -m_max_vel_trans) cmd_vel.linear.y = -m_max_vel_trans;
        if(cmd_vel.angular.z > m_max_vel_rot) cmd_vel.angular.z = m_max_vel_rot;
        if(cmd_vel.angular.z < -m_max_vel_rot) cmd_vel.angular.z = -m_max_vel_rot;

        m_last_cmd = cmd_vel;

        ROS_WARN("[WLP] target_x= %.2f, cmd_x = %.2f",target_vel_x, cmd_vel.linear.x);
        
        return true;
    }


    bool WPV4DiffLocalPlanner::isGoalReached()
    {
        //ROS_WARN("[WLP]isGoalReached() ");
        if (m_goal_reached)
        {
            ROS_INFO("GOAL Reached!");
            return true;
        }
        return false;
    }

    void WPV4DiffLocalPlanner::getTransformedPosition(geometry_msgs::PoseStamped& pose, std::string& frame_id, double& x, double& y, double& theta)
    {
        geometry_msgs::PoseStamped tf_pose;
        pose.header.stamp = ros::Time(0);
        m_tf_listener->transformPose(frame_id, pose, tf_pose);
        x = tf_pose.pose.position.x;
        y = tf_pose.pose.position.y,
        theta = tf::getYaw(tf_pose.pose.orientation);
    }

}