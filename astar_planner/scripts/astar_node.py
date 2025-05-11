#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point
from heapq import heappush, heappop
import tf

class AStarPlanner:
    def __init__(self):
        rospy.init_node('astar_planner')

        self.map_data = None
        self.resolution = 0.0
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.width = 0
        self.height = 0

        self.sub_map = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.sub_goal = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)

        self.path_pub = rospy.Publisher('/path', Path, queue_size=10)

    def map_callback(self, msg):
        self.map_data = msg.data
        self.resolution = msg.info.resolution
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        self.width = msg.info.width
        self.height = msg.info.height

    def world_to_grid(self, x, y):
        gx = int((x - self.origin_x) / self.resolution)
        gy = int((y - self.origin_y) / self.resolution)
        return gx, gy

    def grid_to_world(self, gx, gy):
        x = gx * self.resolution + self.origin_x
        y = gy * self.resolution + self.origin_y
        return x, y

    def is_valid(self, x, y):
        if x < 0 or y < 0 or x >= self.width or y >= self.height:
            return False
        index = y * self.width + x
        return self.map_data[index] == 0  # 0 = free

    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def astar(self, start, goal):
        neighbors = [(0,1),(1,0),(0,-1),(-1,0)]  # 四方向
        close_set = set()
        came_from = {}
        gscore = {start: 0}
        fscore = {start: self.heuristic(start, goal)}
        oheap = []

        heappush(oheap, (fscore[start], start))

        while oheap:
            current = heappop(oheap)[1]

            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                return path[::-1]

            close_set.add(current)

            for dx, dy in neighbors:
                neighbor = (current[0] + dx, current[1] + dy)
                tentative_g_score = gscore[current] + 1

                if not self.is_valid(neighbor[0], neighbor[1]):
                    continue

                if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, float('inf')):
                    continue

                if tentative_g_score < gscore.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    gscore[neighbor] = tentative_g_score
                    fscore[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    heappush(oheap, (fscore[neighbor], neighbor))

        return None  # no path found

    def goal_callback(self, msg):
        if self.map_data is None:
            rospy.logwarn("Map not loaded yet.")
            return

        sx, sy = self.world_to_grid(msg.pose.position.x, msg.pose.position.y)
        rospy.loginfo(f"Goal received at ({sx}, {sy})")

        # 获取当前位置（这里简化为地图中心）
        start = (self.width // 2, self.height // 2)

        path = self.astar(start, (sx, sy))
        if path:
            self.publish_path(path)
        else:
            rospy.logwarn("No path found!")

    def publish_path(self, path):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()

        for point in path:
            x, y = self.grid_to_world(point[0], point[1])
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position = Point(x, y, 0)
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        rospy.loginfo("Published path.")


if __name__ == '__main__':
    planner = AStarPlanner()
    rospy.spin()