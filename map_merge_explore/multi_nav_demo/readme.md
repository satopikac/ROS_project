# priority_multi_nav

优先级多机器人导航分配器（ROS1）。

## 功能
- 终端输入 3 个目标点 `(x y z yaw)`
- 按输入顺序作为优先级
- 贪心分配：
  - 第1个点给最近机器人
  - 第2个点给剩余最近机器人
  - 第3个点给最后一个机器人
- 给 `/tb3_i/move_base` 发送导航目标并等待结果

## 依赖
- ROS Noetic
- move_base 已在每台机器人命名空间下启动（如 `/tb3_0/move_base`）

## 运行
```bash
catkin_make
source devel/setup.bash
rosrun priority_multi_nav priority_goal_dispatcher.py _robots:="tb3_0,tb3_1,tb3_2" _global_frame:="map"