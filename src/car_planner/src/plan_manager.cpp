
#include "car_planner/plan_manager.h"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

void HybridManager::initPlanModules(ros::NodeHandle& nh)
{
    // std::cout << "HybridManager init\n";
    sdf_map_.reset(new SDFMap);
    sdf_map_->initMap(nh);
    edt_environment_.reset(new EDTEnvironment);
    edt_environment_->setMap(sdf_map_);

    kino_path_finder_.reset(new Car_KinoSearch);
    kino_path_finder_->setParam(nh);
    kino_path_finder_->setEnvironment(edt_environment_);
    kino_path_finder_->init();
    std::cout << "HybridManager init done\n";
}

bool HybridManager::checkTrajCollision(double& distance) {
// TODO: 碰撞检测    
    return true;
}

Eigen::Vector3d HybridManager::get_traj_point(double now_time)
{
    // 可能会出现错误!!!!!!!!!!!!!!!

    Eigen::Vector3d current_state;

    // 发送当前时刻的位置
    current_state = kino_path_finder_->evaluate_state(now_time);

    return current_state;
}

bool HybridManager::hybridReplan(Eigen::Vector3d start_state, Eigen::Vector3d end_state)
{
    kino_path_finder_->reset();
    int status = kino_path_finder_->car_search(start_state, end_state);     // 搜索路径

    if (status == Car_KinoSearch::NO_PATH) 
    {
        ROS_WARN("[Hybrid replan]: Can't find path, return.");
        return false;
    } 
    else 
    {
        std::cout <<"[Hybrid replan]: hybrid Search success." << std::endl;
    }

    // kino_path_finder_->visualize(0.1);   // 绘制车体路径
    kino_path_finder_->draw_path(0.05);     // 绘制路径

    traj_duration = kino_path_finder_->get_totalT();

    start_time_ = ros::Time::now(); // 记录搜索起始时间

    return true;
}

