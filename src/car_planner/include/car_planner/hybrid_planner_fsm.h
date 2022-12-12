
#ifndef _HYBRID_PLANNER_FSM_H_
#define _HYBRID_PLANNER_FSM_H_

#include <ros/ros.h>
#include <string.h>
#include <Eigen/Eigen>
#include <vector>
#include <algorithm>
#include <iostream>

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include <visualization_msgs/Marker.h>

#include "car_planner/plan_manager.h"

#include "std_msgs/Float32MultiArray.h"

using std::vector;


class HybridReplanFSM 
{
private:

  /* ---------- flag ---------- */
  enum FSM_EXEC_STATE { INIT, WAIT_TARGET, GEN_NEW_TRAJ, REPLAN_TRAJ, EXEC_TRAJ, REPLAN_NEW };


  HybridManager::Ptr planner_manager_;

  /* parameters */
  double no_replan_thresh_, replan_thresh_;

  /* planning data */
  bool trigger_, have_target_, have_odom_;
  FSM_EXEC_STATE exec_state_;

  Eigen::Vector3d odom_pos_, odom_vel_;  // odometry state
  Eigen::Quaterniond odom_orient_;

  Eigen::Vector3d start_state_;         // start state
  Eigen::Vector3d end_state_;           // target state

  /* ROS utils */
  ros::NodeHandle node_;
  ros::Timer exec_timer_, safety_timer_, vis_timer_, cmd_timer_;
  ros::Subscriber waypoint_sub_, odom_sub_;
  ros::Publisher replan_pub_, new_pub_, traj_pub_;

  ros::Publisher mpc_traj_pub;

  bool plan_success;

  /* helper functions */
  bool callHybridReplan();        // front-end and back-end method

  void changeFSMExecState(FSM_EXEC_STATE new_state, std::string pos_call);
  void printFSMExecState();

  /* ROS functions */
  void execFSMCallback(const ros::TimerEvent& e);
  void checkCollisionCallback(const ros::TimerEvent& e);
  // void waypointCallback(const nav_msgs::PathConstPtr& msg);
  void waypointCallback(const geometry_msgs::PoseStamped& msg);
  void odometryCallback(const nav_msgs::OdometryConstPtr& msg);

  void pubCMDCallback(const ros::TimerEvent& e);


public:
  HybridReplanFSM(/* args */) {
  }
  ~HybridReplanFSM() {
  }

  void init(ros::NodeHandle& nh);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


#endif