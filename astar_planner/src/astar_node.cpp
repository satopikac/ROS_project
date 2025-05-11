#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>  // 新增头文件
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include "astar_planner/astar.h"

class AStarPlannerNode {
public:
    AStarPlannerNode() : nh_("~"), astar_(NULL), initial_x_(-1), initial_y_(-1) {
        map_sub_ = nh_.subscribe("/map", 1, &AStarPlannerNode::mapCallback, this);
        goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &AStarPlannerNode::goalCallback, this);
        path_pub_ = nh_.advertise<nav_msgs::Path>("/path", 10);

        // 订阅 /initialpose
        init_pose_sub_ = nh_.subscribe("/initialpose", 1, &AStarPlannerNode::initPoseCallback, this);
    }

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        map_ = *msg;
        astar_ = new AStar(map_);
        ROS_INFO("Map loaded.");
    }

    void initPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
        double ox = map_.info.origin.position.x;
        double oy = map_.info.origin.position.y;
        double res = map_.info.resolution;

        int px = (msg->pose.pose.position.x - ox) / res;
        int py = (msg->pose.pose.position.y - oy) / res;

        if (px >= 0 && py >= 0 && px < map_.info.width && py < map_.info.height) {
            initial_x_ = px;
            initial_y_ = py;
            ROS_INFO("Initial pose set to grid (%d, %d)", px, py);
        } else {
            ROS_WARN("Initial pose out of map bounds!");
        }
    }

    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        if (!astar_) return;

        double ox = map_.info.origin.position.x;
        double oy = map_.info.origin.position.y;
        double res = map_.info.resolution;

        int gx = (msg->pose.position.x - ox) / res;
        int gy = (msg->pose.position.y - oy) / res;

        // 如果没有设置初始位置，则默认为中心点
        int sx = (initial_x_ != -1) ? initial_x_ : map_.info.width / 2;
        int sy = (initial_y_ != -1) ? initial_y_ : map_.info.height / 2;

        ROS_INFO("Planning from (%d, %d) to (%d, %d)", sx, sy, gx, gy);

        auto path = astar_->plan({sx, sy}, {gx, gy});

        if (!path.empty()) {
            publishPath(path);
        } else {
            ROS_WARN("No path found!");
        }
    }

    void publishPath(const std::vector<std::pair<int, int>>& path) {
        nav_msgs::Path path_msg;
        path_msg.header.frame_id = "map";
        path_msg.header.stamp = ros::Time::now();

        double ox = map_.info.origin.position.x;
        double oy = map_.info.origin.position.y;
        double res = map_.info.resolution;

        for (auto p : path) {
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = p.first * res + ox;
            pose.pose.position.y = p.second * res + oy;
            pose.pose.orientation.w = 1.0;
            pose.header = path_msg.header;
            path_msg.poses.push_back(pose);
        }

        path_pub_.publish(path_msg);
        ROS_INFO("Published path with %lu points.", path.size());
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber map_sub_, goal_sub_, init_pose_sub_;
    ros::Publisher path_pub_;
    nav_msgs::OccupancyGrid map_;
    AStar* astar_;

    int initial_x_, initial_y_;  // 初始位置坐标
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "astar_planner");
    AStarPlannerNode node;
    ros::spin();
    return 0;
}