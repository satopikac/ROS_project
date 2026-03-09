启动多个机器人(提前安装turtlebot3系列)到模拟环境
启动三个机器人各自的slam
启动合并（可以指定初始位置，也可以不指定，网格间的变换由特征匹配算法估计，因此网格之间必须有足够的重叠空间以实现高概率匹配。
）
启动各自的键盘控制节点


map_merge参数细节见 https://wiki.ros.org/multirobot_map_merge#Merging_modes
`
roslaunch turtlebot3_gazebo multi_turtlebot3.launch
roslaunch turtlebot3_gazebo multi_turtlebot3_slam.launch ns:=tb3_0
roslaunch turtlebot3_gazebo multi_turtlebot3_slam.launch ns:=tb3_1
roslaunch turtlebot3_gazebo multi_turtlebot3_slam.launch ns:=tb3_2
roslaunch turtlebot3_gazebo multi_map_merge.launch
ROS_NAMESPACE=tb3_0 rosrun turtlebot3_teleop turtlebot3_teleop_key
ROS_NAMESPACE=tb3_1 rosrun turtlebot3_teleop turtlebot3_teleop_key
ROS_NAMESPACE=tb3_2 rosrun turtlebot3_teleop turtlebot3_teleop_key
`

