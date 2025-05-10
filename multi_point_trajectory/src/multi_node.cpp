#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <stdio.h>
#include <map>
#include <vector>
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>

enum TrajectoryType {
    CIRCLE,
    SINUSOID,
    STRAIGHT_LINE
};

struct PointState {
    float x, y;
};

// 全局变量用于保存按键状态
std::string last_key;

void keyCallback(const std_msgs::String::ConstPtr& msg) {
    last_key = msg->data;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "multi_point_trajectory_node");
    ros::NodeHandle nh;

    // 订阅键盘输入话题
    ros::Subscriber sub = nh.subscribe("/keyboard_input", 10, keyCallback);

    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    ros::Rate rate(10); // 10 Hz

    float t = 0.0;
    TrajectoryType traj_type = CIRCLE;
    
    // 存储轨迹历史
    std::map<std::string, std::vector<geometry_msgs::Point>> trajectory_history;
    
    // 轨迹参数
    float radius = 2.0;
    float speed = 0.1;
    float sin_amplitude = 1.0;
    float sin_frequency = 1.0;
    
    // 轨迹范围限制
    float x_min = -5.0;
    float x_max = 5.0;
    bool direction = true; // true为正向，false为反向
    
    // 多个小球的颜色和相位差
    std::vector<std::pair<std::string, std::pair<float, float>>> points = {
        {"red_ball", {1.0, 0.0}},   // 红球
        {"green_ball", {0.0, 1.0}}, // 绿球
        {"blue_ball", {0.0, 0.0}}   // 蓝球
    };
    
    // 相位差设置
    std::vector<float> phase_offsets = {0.0, 2.0 * M_PI / 3.0, 4.0 * M_PI / 3.0}; // 相差120度
    
    // 初始化轨迹历史
    for (auto& point : points) {
        trajectory_history[point.first] = std::vector<geometry_msgs::Point>();
    }

    while (ros::ok())
    {
        std::map<std::string, visualization_msgs::Marker> markers;

        // 创建轨迹线
        for (size_t i = 0; i < points.size(); ++i) {
            auto& point = points[i];
            visualization_msgs::Marker line_marker;
            line_marker.header.frame_id = "map";
            line_marker.header.stamp = ros::Time::now();
            line_marker.ns = "trajectory_" + point.first;
            line_marker.id = 0;
            line_marker.type = visualization_msgs::Marker::LINE_STRIP;
            line_marker.action = visualization_msgs::Marker::ADD;
            line_marker.scale.x = 0.05;
            line_marker.color.r = point.second.first;
            line_marker.color.g = point.second.second;
            line_marker.color.b = (i == 2) ? 1.0 : 0.0;
            line_marker.color.a = 0.7;
            line_marker.pose.orientation.w = 1.0;
            line_marker.points = trajectory_history[point.first];
            markers["line_" + point.first] = line_marker;
        }

        // 创建小球
        for (size_t i = 0; i < points.size(); ++i) {
            auto& point = points[i];
            float phase_offset = phase_offsets[i];

            visualization_msgs::Marker ball_marker;
            ball_marker.header.frame_id = "map";
            ball_marker.header.stamp = ros::Time::now();
            ball_marker.ns = point.first;
            ball_marker.id = 0;
            ball_marker.type = visualization_msgs::Marker::SPHERE;
            ball_marker.action = visualization_msgs::Marker::ADD;

            ball_marker.scale.x = 0.2;
            ball_marker.scale.y = 0.2;
            ball_marker.scale.z = 0.2;

            ball_marker.color.r = point.second.first;
            ball_marker.color.g = point.second.second;
            ball_marker.color.b = (i == 2) ? 1.0 : 0.0;
            ball_marker.color.a = 1.0;

            float x, y;

            switch (traj_type) {
                case CIRCLE:
                    x = radius * cos(t + phase_offset);
                    y = radius * sin(t + phase_offset);
                    break;
                case SINUSOID:
                    x = t + i * 0.5;
                    y = sin_amplitude * sin(sin_frequency * x + phase_offset);
                    break;
                case STRAIGHT_LINE:
                    x = t + i * 0.5;
                    y = i - 1.0;
                    break;
            }

            ball_marker.pose.position.x = x;
            ball_marker.pose.position.y = y;
            ball_marker.pose.position.z = 0.1;
            ball_marker.pose.orientation.w = 1.0;
            ball_marker.lifetime = ros::Duration();

            markers[point.first] = ball_marker;

            geometry_msgs::Point p;
            p.x = x;
            p.y = y;
            p.z = 0.1;

            if (trajectory_history[point.first].size() > 500) {
                trajectory_history[point.first].erase(trajectory_history[point.first].begin());
            }
            trajectory_history[point.first].push_back(p);
        }

        // 发布所有 Marker
        for (auto& pair : markers) {
            marker_pub.publish(pair.second);
        }

        // 处理按键事件
        if (!last_key.empty()) {
            switch (last_key[0]) {
                case 'c':
                    traj_type = CIRCLE;
                    t = 0.0;
                    for (auto& point : points) trajectory_history[point.first].clear();
                    ROS_INFO("Switched to CIRCLE trajectory.");
                    break;
                case 's':
                    traj_type = SINUSOID;
                    t = x_min;
                    for (auto& point : points) trajectory_history[point.first].clear();
                    ROS_INFO("Switched to SINUSOID trajectory.");
                    break;
                case 'l':
                    traj_type = STRAIGHT_LINE;
                    t = x_min;
                    for (auto& point : points) trajectory_history[point.first].clear();
                    ROS_INFO("Switched to STRAIGHT LINE trajectory.");
                    break;
                case '+':
                    speed *= 1.2;
                    ROS_INFO("Speed increased to %.2f", speed);
                    break;
                case '-':
                    speed *= 0.8;
                    ROS_INFO("Speed decreased to %.2f", speed);
                    break;
            }
            last_key.clear();  // 清空已处理的按键
        }

        // 更新时间参数
        if (traj_type == CIRCLE) {
            t += speed;
        } else {
            if (direction) {
                t += speed;
                if (t >= x_max) direction = false;
            } else {
                t -= speed;
                if (t <= x_min) direction = true;
            }
        }

        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}