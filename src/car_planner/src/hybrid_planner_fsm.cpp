

#include "car_planner/hybrid_planner_fsm.h"

void HybridReplanFSM::init(ros::NodeHandle& nh) 
{
    exec_state_  = FSM_EXEC_STATE::INIT;
    have_target_ = false;
    have_odom_   = false;
    plan_success = false;

    /*  参数设置  */
    nh.param("fsm/thresh_replan", replan_thresh_, -1.0);
    nh.param("fsm/thresh_no_replan", no_replan_thresh_, -1.0);
    nh.param("car_search/car_l",car_l,0.6);
    nh.param("car_search/car_w",car_w,0.4);
    nh.param("car_search/car_h",car_h,0.3);

    // 规划管理：主要的模块 -----------------------------
    planner_manager_.reset(new HybridManager);
    planner_manager_->initPlanModules(nh);
    // ----------------------------------------------

    // 显示模块
    // visualization_.reset(new PlanningVisualization(nh));


    /* callback */
    exec_timer_   = nh.createTimer(ros::Duration(0.10), &HybridReplanFSM::execFSMCallback, this);
    // safety_timer_ = nh.createTimer(ros::Duration(0.05), &HybridReplanFSM::checkCollisionCallback, this);

    cmd_timer_ = nh.createTimer(ros::Duration(0.05), &HybridReplanFSM::pubRefTrajCallback, this);

    waypoint_sub_ =
        nh.subscribe("waypoints", 1, &HybridReplanFSM::waypointCallback, this);


    odom_sub_ = nh.subscribe("/odom_world", 1, &HybridReplanFSM::odometryCallback, this);

    mpc_traj_pub = nh.advertise<std_msgs::Float32MultiArray>("mpc/traj_point", 50);

}


void HybridReplanFSM::waypointCallback(const geometry_msgs::PoseStamped& msg)
{
    std::cout << "Triggered!" << std::endl;
    trigger_ = true;
    Eigen::Quaterniond tq; 
    tq.x() = msg.pose.orientation.x;
    tq.y() = msg.pose.orientation.y;
    tq.z() = msg.pose.orientation.z;
    tq.w() = msg.pose.orientation.w;
    Eigen::Matrix3d trot(tq);
    double tyaw  = atan2(trot.col(0)[1],trot.col(0)[0]);

    end_state_ << msg.pose.position.x, msg.pose.position.y, tyaw;

    ROS_INFO_STREAM("TARGET=" << end_state_.transpose());
    ROS_INFO("[node] receive the planning target");
    have_target_ = true;

    if (exec_state_ == WAIT_TARGET)
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
    else if (exec_state_ == EXEC_TRAJ)
        changeFSMExecState(REPLAN_TRAJ, "TRIG");

}


void HybridReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr& msg)
{
  odom_pos_(0) = msg->pose.pose.position.x;
  odom_pos_(1) = msg->pose.pose.position.y;
  odom_pos_(2) = msg->pose.pose.position.z;

  odom_vel_(0) = msg->twist.twist.linear.x;
  odom_vel_(1) = msg->twist.twist.linear.y;
  odom_vel_(2) = msg->twist.twist.linear.z;

  odom_orient_.w() = msg->pose.pose.orientation.w;
  odom_orient_.x() = msg->pose.pose.orientation.x;
  odom_orient_.y() = msg->pose.pose.orientation.y;
  odom_orient_.z() = msg->pose.pose.orientation.z;

  have_odom_ = true;
}


void HybridReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, std::string pos_call)
{
  std::string state_str[5] = { "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ" };
  int    pre_s        = int(exec_state_);
  exec_state_         = new_state;
  std::cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << std::endl;
}


void HybridReplanFSM::printFSMExecState()
{
    std::string state_str[5] = { "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ" };

    std::cout << "[FSM]: state: " + state_str[int(exec_state_)] << std::endl;
}


void HybridReplanFSM::execFSMCallback(const ros::TimerEvent& e)
{
    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100) {
        printFSMExecState();
        if (!have_odom_) std::cout << "no odom." << std::endl;
        if (!trigger_) std::cout << "wait for goal." << std::endl;
        fsm_num = 0;
    }

    switch (exec_state_)
    {
        case INIT: 
        {
            if (!have_odom_){return;}
            if (!trigger_){return;}
            changeFSMExecState(WAIT_TARGET, "FSM");
            break;
        }

        case WAIT_TARGET: 
        {
            if (!have_target_){return;}
            else 
            {
                // planner_manager_->SetMap();
                changeFSMExecState(GEN_NEW_TRAJ, "FSM");
            }
            break;
        }

        case GEN_NEW_TRAJ: 
        {
            // TODO:
            // 规划全新轨迹

            Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);

            start_state_ << odom_pos_[0], odom_pos_[1], atan2(rot_x(1), rot_x(0));  // 设置规划的起点为机器人定位

            bool success = callHybridReplan();  // 规划
            if (success) 
            {
                changeFSMExecState(EXEC_TRAJ, "FSM");
            } 
            else 
            {
                changeFSMExecState(GEN_NEW_TRAJ, "FSM");
            }
            break;
        }

        case EXEC_TRAJ: 
        {
            // TODO:
            // 离线轨迹执行模式
            ros::Time start_time_ = planner_manager_->start_time_;  // 规划的起始时间

            double traj_duration = planner_manager_->traj_duration; // 轨迹持续时间

            ros::Time time_now = ros::Time::now();
            double t_cur = (time_now - start_time_).toSec();        // 相对于规划起始时刻的时间

            Eigen::Vector2d start_pt_, end_pt_, odom_pt_;
            start_pt_ << start_state_[0], start_state_[1];
            end_pt_ << end_state_[0], end_state_[1];
            odom_pt_ << odom_pos_[0], odom_pos_[1];

            if(t_cur > traj_duration)
            {
                have_target_ = false;
                changeFSMExecState(WAIT_TARGET, "FSM");
                return;
            }
            else if ((end_pt_ - odom_pt_).norm() < no_replan_thresh_) {
                // cout << "near end" << endl;
                return;

            } else if ((start_pt_ - odom_pt_).norm() < replan_thresh_) {
                // cout << "near start" << endl;
                return;

            } else {
                changeFSMExecState(REPLAN_TRAJ, "FSM");
            }
            break;
        }
        
        case REPLAN_TRAJ: 
        {
            // TODO:
            // 重规划模式：根据现有轨迹状态，规划轨迹

            ros::Time start_time_ = planner_manager_->start_time_;  // 规划的起始时间
            ros::Time time_now = ros::Time::now();
            double t_cur = (time_now - start_time_).toSec();        // 相对于规划起始时刻的时间

            start_state_ = planner_manager_->get_traj_point(t_cur);

            bool success = callHybridReplan();
            if (success) {
                changeFSMExecState(EXEC_TRAJ, "FSM");
            } else {
                changeFSMExecState(GEN_NEW_TRAJ, "FSM");
            }
            break;
        }

    }

}

// TODO: 碰撞检测的回调
void HybridReplanFSM::checkCollisionCallback(const ros::TimerEvent& e)
{
    if(have_target_) {
        Eigen::Vector3d target_pt;
        target_pt << end_state_[0], end_state_[1], 0.2;
        auto map_env = planner_manager_->edt_environment_;
        std::cout << "11111111111111\n";
        double target_to_obs = map_env->evaluateCoarseEDT(target_pt, -1.0);
        std::cout << "222222222222222\n";

        if(target_to_obs <= 0.5){
            const double    dr = 0.5, dtheta = 30;
            double          max_dist = -1.0;
            Eigen::Vector3d new_goal, tmp_goal;
            // 搜索目标点附近的可行点作为新的目标点 
            std::cout << "333333333333333333\n";

            for (double r = dr; r <= 5 * dr + 1e-3; r += dr) {
                for (double theta = -90; theta <= 270; theta += dtheta) {
                    tmp_goal[0] = target_pt(0) + r * cos(theta / 57.3);
                    tmp_goal[1] = target_pt(1) + r * sin(theta / 57.3);
                    tmp_goal[2] = 0.2;
                    target_to_obs = map_env->evaluateCoarseEDT(tmp_goal, -1.0);

                    if(target_to_obs > max_dist){
                        max_dist = target_to_obs;
                        new_goal = tmp_goal;
                    }
                }
            }
            std::cout << "444444444444444444\n";

            // 如果找到了该可行点
            if (max_dist > 0.5) {
                cout << "change goal, replan." << endl;
                end_state_[0] = new_goal[0];
                end_state_[1] = new_goal[1];

                have_target_ = true;

                if (exec_state_ == EXEC_TRAJ) {
                    changeFSMExecState(REPLAN_TRAJ, "SAFETY");
                }
            } 
            // 没有找到可行点
            else {

                cout << "goal near collision, keep retry" << endl;
                changeFSMExecState(REPLAN_TRAJ, "FSM");
            }
        }
        else{
            std::cout << "555555555555\n";
        }
    }
}


void HybridReplanFSM::pubRefTrajCallback(const ros::TimerEvent& e)
{
    // 拿到规划成功的轨迹
    // std::cout << "pub cmd in\n";
    if(!plan_success)   // 没有规划成功。返回
    {
        return;
    }

    ros::Time start_time_ = planner_manager_->start_time_;  // 规划的起始时间

    double traj_duration = planner_manager_->traj_duration; // 轨迹持续时间

    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - start_time_).toSec();        // 相对于规划起始时刻的时间

    Eigen::Vector3d traj_pt;                                // 参考轨迹点

    std_msgs::Float32MultiArray waypoint_array;             // 参考轨迹消息类型

    double delta_t = 0.05;

    for(int i = 0; i < 10; i++)
    {
        double t_add = t_cur + i * delta_t;

        if (t_add < traj_duration && t_cur >= 0.0) 
        {
            traj_pt = planner_manager_->get_traj_point(t_add);
        }
        else if(t_add >= traj_duration)
        {
            traj_pt = planner_manager_->get_traj_point(traj_duration);
        }
        else
        {
            cout << "[HybridReplanFSM]: invalid time to pub." << endl;
        }
        // 拿到 X, Y, Theta
        waypoint_array.data.push_back(traj_pt[0]);      // X
        waypoint_array.data.push_back(traj_pt[1]);      // Y
        waypoint_array.data.push_back(traj_pt[2]);      // Theta
    }

    mpc_traj_pub.publish(waypoint_array);        // 发送参考轨迹    
    // std::cout << "pub cmd done\n";
}


bool HybridReplanFSM::callHybridReplan()
{
    std::cout << "[HybridReplanFSM]: Try to generate new traj." << std::endl;
    clock_t time_start,time_end;  //定义clock_t变量用于计时
    time_start = clock();          //搜索开始时间

    plan_success = planner_manager_->hybridReplan(start_state_, end_state_);

    time_end = clock();   //搜索结束时间

    double t_search = double(time_end-time_start)/CLOCKS_PER_SEC;
    
    std::cout << "[HybridReplanFSM]: Hybrid Astar cost time:= " << t_search << std::endl;


    if(plan_success)
    {
        std::cout << "[HybridReplanFSM]: generate new traj success." << std::endl;
        return true;
    }
    else
    {
        std::cout << "[HybridReplanFSM]: generate new traj fail." << std::endl;
        return false;
    }
}
