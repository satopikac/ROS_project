#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import queue
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

        self.lock = threading.Lock()
        self.clients = {}
        self.results = {}
        self.goal_queues = {}
        self.active_robots = set()
        self.stop_event = threading.Event()

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
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            rospy.logwarn("TF lookup failed for %s -> %s: %s", self.global_frame, base_frame, str(e))
            return None

    @staticmethod
    def dist2d(p1, p2):
        return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

    def parse_goal_text(self, raw):
        vals = [float(v) for v in raw.split()]
        if len(vals) != 4:
            raise ValueError("need 4 values")
        x, y, z, yaw = vals
        if self.yaw_in_degrees:
            yaw = math.radians(yaw)
        return (x, y, z, yaw)

    def read_one_goal(self, prompt_text):
        while not rospy.is_shutdown():
            try:
                raw = input(prompt_text).strip()
                return self.parse_goal_text(raw)
            except ValueError:
                print("格式错误，请输入: x y z yaw，例如 1.0 2.0 0.0 1.57")
        return None

    def read_goals_from_user(self, n=3):
        goals = []
        for i in range(n):
            goal = self.read_one_goal(f"请输入第 {i + 1} 个目标点 (x y z yaw): ")
            if goal is None:
                break
            goals.append(goal)
        return goals

    def assign_goals_priority_greedy(self, goals):
        remaining_robots = list(self.robots[:3])
        assignments = []

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
                raise RuntimeError(f"优先级第 {idx + 1} 个目标无法分配：剩余机器人 TF 不可用。")

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

        goal = MoveBaseGoal()
        goal.target_pose = PoseStamped()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = self.global_frame
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = z
        goal.target_pose.pose.orientation.x = qx
        goal.target_pose.pose.orientation.y = qy
        goal.target_pose.pose.orientation.z = qz
        goal.target_pose.pose.orientation.w = qw
        return goal

    def worker(self, robot):
        ns = f"/{robot}/move_base"
        client = actionlib.SimpleActionClient(ns, MoveBaseAction)

        with self.lock:
            self.clients[robot] = client
            self.active_robots.add(robot)

        rospy.loginfo("[%s] waiting for move_base server...", robot)
        if not client.wait_for_server(rospy.Duration(self.server_timeout)):
            rospy.logerr("[%s] move_base server timeout", robot)
            with self.lock:
                self.results[robot] = "SERVER_TIMEOUT"
                self.active_robots.discard(robot)
            return

        while not rospy.is_shutdown():
            try:
                goal = self.goal_queues[robot].get(timeout=0.5)
            except queue.Empty:
                with self.lock:
                    if robot not in self.active_robots:
                        break
                continue

            goal_msg = self.build_goal_msg(goal)
            rospy.loginfo("[%s] send goal: x=%.2f y=%.2f z=%.2f yaw=%.2f", robot, goal[0], goal[1], goal[2], goal[3])
            client.send_goal(goal_msg)

            finished = client.wait_for_result(rospy.Duration(self.goal_timeout))
            if not finished:
                client.cancel_goal()
                rospy.logwarn("[%s] goal timeout, canceled", robot)
                with self.lock:
                    self.results[robot] = "GOAL_TIMEOUT"
                    if self.goal_queues[robot].empty():
                        self.active_robots.discard(robot)
                continue

            state = client.get_state()
            state_text = STATUS_TEXT.get(state, f"STATE_{state}")
            rospy.loginfo("[%s] result: %s", robot, state_text)

            with self.lock:
                self.results[robot] = state_text

            if state == GoalStatus.SUCCEEDED:
                with self.lock:
                    if self.goal_queues[robot].empty():
                        self.active_robots.discard(robot)
            elif state in (GoalStatus.PREEMPTED, GoalStatus.RECALLED):
                rospy.loginfo("[%s] current goal skipped, waiting for next goal...", robot)
            else:
                with self.lock:
                    if self.goal_queues[robot].empty():
                        self.active_robots.discard(robot)

        rospy.loginfo("[%s] worker exit", robot)

    def command_loop(self):
        while not rospy.is_shutdown() and not self.stop_event.is_set():
            try:
                cmd = input("输入命令，s=跳过并重设目标，q=退出监听: ").strip()
            except EOFError:
                return

            if cmd == "q":
                rospy.loginfo("命令监听已退出")
                return

            if cmd != "s":
                print("未知命令。可用命令: s, q")
                continue

            with self.lock:
                active = sorted(self.active_robots)

            if not active:
                print("当前没有正在执行任务的机器人。")
                continue

            print("当前可跳过任务的机器人: " + ", ".join(active))
            robot = input("请输入要跳过当前任务的机器人名: ").strip()

            with self.lock:
                client = self.clients.get(robot)
                is_active = robot in self.active_robots

            if client is None or not is_active:
                print("该机器人当前没有活动任务。")
                continue

            new_goal = self.read_one_goal(f"请输入 {robot} 的新目标点 (x y z yaw): ")
            if new_goal is None:
                continue

            rospy.loginfo("[%s] cancel current goal and assign new goal", robot)
            client.cancel_goal()
            self.goal_queues[robot].put(new_goal)

    def execute_assignments_parallel(self, assignments):
        threads = []

        for robot, goal in assignments:
            self.goal_queues[robot] = queue.Queue()
            self.goal_queues[robot].put(goal)

        cmd_thread = threading.Thread(target=self.command_loop, daemon=True)
        cmd_thread.start()

        for robot, _ in assignments:
            thread = threading.Thread(target=self.worker, args=(robot,), daemon=True)
            threads.append(thread)
            thread.start()

        for thread in threads:
            thread.join()

        self.stop_event.set()
        return dict(self.results)


def main():
        rospy.init_node("priority_goal_dispatcher", anonymous=False)

        app = PriorityMultiNav()
        rospy.loginfo("Robots: %s", ", ".join(app.robots[:3]))
        rospy.loginfo("Global frame: %s", app.global_frame)
        rospy.loginfo("导航过程中可输入 s，跳过某台机器人的当前任务并重新输入目标点")

        goals = app.read_goals_from_user(n=3)
        assignments = app.assign_goals_priority_greedy(goals)
        results = app.execute_assignments_parallel(assignments)

        rospy.loginfo("=== Final Results ===")
        for robot, status in results.items():
            rospy.loginfo("%s -> %s", robot, status)


if __name__ == "__main__":
    main()