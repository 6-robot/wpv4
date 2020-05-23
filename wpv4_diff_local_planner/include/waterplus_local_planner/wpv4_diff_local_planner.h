
#ifndef WPV4_DIFF_LOCAL_PLANNER_H_
#define WPV4_DIFF_LOCAL_PLANNER_H_

#include <ros/ros.h>

// base local planner base class and utilities
#include <nav_core/base_local_planner.h>
#include <base_local_planner/goal_functions.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <base_local_planner/costmap_model.h>

// costmap
#include <costmap_2d/costmap_2d_ros.h>

// message types
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>

// transforms
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#define WLP_STEP_GOTO     1
#define WLP_STEP_NEAR     2
#define WLP_STEP_ARRIVED  3

namespace waterplus_local_planner
{
  class WPV4DiffLocalPlanner : public nav_core::BaseLocalPlanner
  {
  public:
    WPV4DiffLocalPlanner();
    ~WPV4DiffLocalPlanner();

    void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros);
    void initialize(std::string name,tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);
    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);
    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
    bool isGoalReached();
    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan);

  protected:
    void getTransformedPosition(geometry_msgs::PoseStamped& pose, std::string& frame_id, double& x, double& y, double& theta);
    
  private:
    bool m_bInitialized;
    int m_nStep;
    int m_nPathIndex;

    costmap_2d::Costmap2DROS* m_costmap_ros;
    costmap_2d::Costmap2D* m_costmap;
    tf::TransformListener* m_tf_listener;
    std::vector<geometry_msgs::PoseStamped> m_global_plan;
    std::string m_global_frame_id;      //!< The frame in which the controller will run
    std::string m_robot_base_frame_id;  //!< Used as the base frame id of the robot
    bool m_goal_reached;
    
    geometry_msgs::Twist m_last_cmd;
    
    std::vector<geometry_msgs::Point> m_footprint_spec;
    double m_robot_inscribed_radius;      //!< The radius of the inscribed circle of the robot (collision possible)
    double m_robot_circumscribed_radius;  //!< The radius of the circumscribed circle of the robot

    bool m_bAC;
    bool m_bFirstStep;
    
    ros::Subscriber m_scan_sub;
    ros::Publisher m_pub_target;
    double m_max_vel_trans;
    double m_max_vel_rot;
    double m_max_acc_trans;
    double m_max_acc_rot;
    double m_acc_scale_trans;
    double m_acc_scale_rot;
    double m_path_dist_tolerance;
    double m_goal_dist_tolerance;
    double m_goal_yaw_tolerance;
  };
}; // end namespace waterplus_local_planner

#endif // WPV4_DIFF_LOCAL_PLANNER_H_