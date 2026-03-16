#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import threading

import rospy
import actionlib
import tf2_ros

from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus


STATUS_TEXT = {
    GoalStatus.PENDING: "PENDING",
    GoalStatus.ACTIVE: "ACTIVE",
    GoalStatus.PREEMPTED: "PREEMPTED",
    GoalStatus.SUCCEEDED: "SUCCEEDED",
    GoalStatus.ABORTED: "ABORTED",
    GoalStatus.REJECTED: "REJECTED",
    GoalStatus.PREEMPTING: "PREEMPTING",
    GoalStatus.RECALLING: "RECALLING",
    GoalStatus.RECALLED: "RECALLED",
    GoalStatus.LOST: "LOST",
}


class PriorityMultiNav:
    def __init__(self):
        robots_str = rospy.get_param("~robots", "tb3_0,tb3_1,tb3_2")
        self.robots = [r.strip() for r in robots_str.split(",") if r.strip()]
        self.global_frame = rospy.get_param("~global_frame", "map")
        self.base_frame_suffix = rospy.get_param("~base_frame_suffix", "base_footprint")
        self.tf_timeout = rospy.get_param("~tf_timeout", 0.5)
        self.server_timeout = rospy.get_param("~server_timeout", 10.0)
        self.goal_timeout = rospy.get_param("~goal_timeout", 300.0)
        self.yaw_in_degrees = rospy.get_param("~yaw_in_degrees", False)

        if len(self.robots) < 3:
            raise RuntimeError("Need at least 3 robots in ~robots parameter.")

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def _robot_base_frame(self, robot):
        return f"{robot}/{self.base_frame_suffix}"

    def get_robot_xy(self, robot):
        base_frame = self._robot_base_frame(robot)
        try:
            trans = self.tf_buffer.lookup_transform(
                self.global_frame,
                base_frame,
                rospy.Time(0),
                rospy.Duration(self.tf_timeout)
            )
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            return x, y
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("TF lookup failed for %s -> %s: %s", self.global_frame, base_frame, str(e))
            return None

    @staticmethod
    def dist2d(p1, p2):
        return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

    def read_goals_from_user(self, n=3):
        goals = []
        for i in range(n):
            while not rospy.is_shutdown():
                try:
                    raw = input(f"请输入第 {i+1} 个目标点 (x y z yaw): ").strip()
                    vals = [float(v) for v in raw.split()]
                    if len(vals) != 4:
                        print("格式错误，请输入 4 个数: x y z yaw")
                        continue
                    x, y, z, yaw = vals
                    if self.yaw_in_degrees:
                        yaw = math.radians(yaw)
                    goals.append((x, y, z, yaw))
                    break
                except ValueError:
                    print("格式错误，请输入数字，例如: 1.0 2.0 0.0 1.57")
        return goals

    def assign_goals_priority_greedy(self, goals):
        remaining_robots = list(self.robots[:3])  # 只取3台
        assignments = []  # [(robot, goal)]

        for idx, goal in enumerate(goals):
            goal_xy = (goal[0], goal[1])

            best_robot = None
            best_dist = float("inf")

            for robot in remaining_robots:
                robot_xy = self.get_robot_xy(robot)
                if robot_xy is None:
                    continue
                d = self.dist2d(robot_xy, goal_xy)
                if d < best_dist:
                    best_dist = d
                    best_robot = robot

            if best_robot is None:
                raise RuntimeError(f"优先级第 {idx+1} 个目标无法分配：剩余机器人 TF 不可用。")

            assignments.append((best_robot, goal))
            remaining_robots.remove(best_robot)

            rospy.loginfo(
                "优先级 %d 目标(%.2f, %.2f) -> %s (距离 %.2f m)",
                idx + 1, goal[0], goal[1], best_robot, best_dist
            )

        return assignments

    def build_goal_msg(self, goal_tuple):
        x, y, z, yaw = goal_tuple
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)

        g = MoveBaseGoal()
        g.target_pose = PoseStamped()
        g.target_pose.header.stamp = rospy.Time.now()
        g.target_pose.header.frame_id = self.global_frame
        g.target_pose.pose.position.x = x
        g.target_pose.pose.position.y = y
        g.target_pose.pose.position.z = z
        g.target_pose.pose.orientation.x = qx
        g.target_pose.pose.orientation.y = qy
        g.target_pose.pose.orientation.z = qz
        g.target_pose.pose.orientation.w = qw
        return g

    def execute_assignments_parallel(self, assignments):
        clients = {}
        results = {}
        threads = []

        def worker(robot, goal):
            ns = f"/{robot}/move_base"
            client = actionlib.SimpleActionClient(ns, MoveBaseAction)
            clients[robot] = client

            rospy.loginfo("[%s] waiting for move_base server...", robot)
            if not client.wait_for_server(rospy.Duration(self.server_timeout)):
                rospy.logerr("[%s] move_base server timeout", robot)
                results[robot] = "SERVER_TIMEOUT"
                return

            goal_msg = self.build_goal_msg(goal)
            rospy.loginfo("[%s] send goal: x=%.2f y=%.2f yaw=%.2f", robot, goal[0], goal[1], goal[3])
            client.send_goal(goal_msg)

            finished = client.wait_for_result(rospy.Duration(self.goal_timeout))
            if not finished:
                client.cancel_goal()
                results[robot] = "GOAL_TIMEOUT"
                rospy.logwarn("[%s] goal timeout, canceled", robot)
                return

            state = client.get_state()
            results[robot] = STATUS_TEXT.get(state, f"STATE_{state}")
            rospy.loginfo("[%s] result: %s", robot, results[robot])

        for robot, goal in assignments:
            t = threading.Thread(target=worker, args=(robot, goal), daemon=True)
            threads.append(t)
            t.start()

        for t in threads:
            t.join()

        return results


def main():
    rospy.init_node("priority_goal_dispatcher", anonymous=False)

    app = PriorityMultiNav()
    rospy.loginfo("Robots: %s", ", ".join(app.robots[:3]))
    rospy.loginfo("Global frame: %s", app.global_frame)

    goals = app.read_goals_from_user(n=3)
    assignments = app.assign_goals_priority_greedy(goals)
    results = app.execute_assignments_parallel(assignments)

    rospy.loginfo("=== Final Results ===")
    for robot, status in results.items():
        rospy.loginfo("%s -> %s", robot, status)


if __name__ == "__main__":
    main()
    