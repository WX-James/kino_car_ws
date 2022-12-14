# kino_car_ws
# 阿克曼小车局部路径规划器

## 依赖安装
（1）casadi求解器安装
```bash
pip install casadi
```
（2）Hunter仿真相关依赖
请参考松灵官方：https://github.com/agilexrobotics/ugv_gazebo_sim

## 编译以使用VSCODE IDE
```bash
catkin_make -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=Yes
```

## 启动

1. Hybrid-Astar全局路径搜索测试

```bash
source devel/setup.bash
roslaunch car_planner car.launch
```

2. Hybrid-Astar局部路径搜索和MPC路径跟踪

```bash
source devel/setup.bash
roslaunch ausim_test hybrid_plan.launch
```
