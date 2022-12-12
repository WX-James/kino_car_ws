#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_


#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include "car_planner/hybrid_search.h"

#include "plan_env/edt_environment.h"

using namespace hybrid_planner;
using namespace fast_planner;



class HybridManager
{
    
public:
    HybridManager(){};
    ~HybridManager(){};

    /* main planning interface */
    bool hybridReplan(Eigen::Vector3d start_state, Eigen::Vector3d end_state);

    void initPlanModules(ros::NodeHandle& nh);

    bool checkTrajCollision(double& distance);

    Eigen::Vector3d get_traj_point(double now_time);

    ros::Time start_time_;  // 搜索起始时间

    double traj_duration;

    vector<Eigen::Vector3d> kino_path_;

    EDTEnvironment::Ptr edt_environment_;

private:

    //TODO:
    

    SDFMap::Ptr sdf_map_;
    unique_ptr<Car_KinoSearch> kino_path_finder_;


public:

    typedef std::unique_ptr<HybridManager> Ptr;

};



#endif