

#include <ros/ros.h>

#include "car_planner/hybrid_planner_fsm.h"




int main(int argc, char** argv) {
    ros::init(argc, argv, "Hybrid_local_planner_node");
    ros::NodeHandle nh("~");

    int planner;
    nh.param("hybrid_planner_node/planner", planner, -1);

    // TODO:
    // 创建 FSM对象----------------------------------------------------------

    std::cout << "Hybrid_local_planner_node init" << std::endl;

    HybridReplanFSM hybrid_replan;

    hybrid_replan.init(nh);

    // --------------------------------------------------------------------

    ros::Duration(1.0).sleep();
    ros::spin();

    return 0;
}
