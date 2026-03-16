#!/usr/bin/env python3
"""
增强版 NavigationManager - 可获取目标执行结果

功能：
- JSON任务连续导航
- 独立位置输出线程
- 独立命令输入线程（s跳过, q停止）
- 每个目标执行完记录结果
- 任务完成可通过属性获取整个任务结果
"""

import rospy
import actionlib
import tf
import threading
import json
from queue import Queue

from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

class NavigationManager:
    def __init__(self):
        if not rospy.core.is_initialized():
            rospy.init_node("navigation_task_manager_json_result", anonymous=True)

        rospy.loginfo("Waiting for move_base server...")
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base")

        self.listener = tf.TransformListener()
        self.goal_queue = Queue()

        # 控制标志
        self.stop_task = False
        self.skip_current = False

        # 每个目标结果列表
        self.goal_results = []

        # 任务完成事件
        self.task_done_event = threading.Event()

        # 启动控制线程
        threading.Thread(target=self._console_control_thread, daemon=True).start()
        # 启动机器人位姿输出线程
        threading.Thread(target=self._robot_pose_thread, daemon=True).start()

    # 获取机器人位姿
    def get_robot_pose(self):
        try:
            (trans, rot) = self.listener.lookupTransform("map", "base_link", rospy.Time(0))
            yaw = tf.transformations.euler_from_quaternion(rot)[2]
            return trans[0], trans[1], yaw
        except:
            return None

    # 位姿输出线程
    def _robot_pose_thread(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            pose = self.get_robot_pose()
            if pose:
                x, y, yaw = pose
                rospy.loginfo("[Robot Pose] x=%.2f y=%.2f yaw=%.2f", x, y, yaw)
            rate.sleep()

    # 控制输入线程
    def _console_control_thread(self):
        while not rospy.is_shutdown():
            cmd = input("\nEnter command ('s'=skip current, 'q'=stop all): ").lower()
            if cmd == 's':
                self.skip_current = True
            elif cmd == 'q':
                self.stop_task = True
                break

    # 执行单个目标
    def execute_goal(self, pose_stamped):
        self.skip_current = False

        goal = MoveBaseGoal()
        goal.target_pose = pose_stamped

        rospy.loginfo("Sending goal x=%.2f y=%.2f", pose_stamped.pose.position.x, pose_stamped.pose.position.y)
        self.client.send_goal(goal)

        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            if self.stop_task:
                rospy.logwarn("Stopping entire task!")
                self.client.cancel_goal()
                return "STOPPED"
            if self.skip_current:
                rospy.logwarn("Skipping current goal")
                self.client.cancel_goal()
                return "SKIPPED"

            state = self.client.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal reached successfully")
                return "SUCCEEDED"
            elif state in [GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.PREEMPTED]:
                rospy.logwarn("Navigation failed")
                return "FAILED"

            rate.sleep()

    # 添加 JSON 任务
    def add_json_task(self, json_data):
        data = json.loads(json_data)
        for p in data["goals"]:
            x, y, yaw = p["x"], p["y"], p["yaw"]
            q = tf.transformations.quaternion_from_euler(0, 0, yaw)

            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "map"
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y
            pose_stamped.pose.position.z = 0
            pose_stamped.pose.orientation.x = q[0]
            pose_stamped.pose.orientation.y = q[1]
            pose_stamped.pose.orientation.z = q[2]
            pose_stamped.pose.orientation.w = q[3]

            self.goal_queue.put(pose_stamped)

        rospy.loginfo("JSON task added, %d goals", len(data["goals"]))

        # 重置任务完成事件
        self.task_done_event.clear()

    # 主循环
    def run(self):
        while not rospy.is_shutdown() and not self.stop_task:
            if self.goal_queue.empty():
                rospy.sleep(0.2)
                continue

            pose_stamped = self.goal_queue.get()
            result = self.execute_goal(pose_stamped)

            self.goal_results.append({
                "x": pose_stamped.pose.position.x,
                "y": pose_stamped.pose.position.y,
                "yaw": tf.transformations.euler_from_quaternion([
                    pose_stamped.pose.orientation.x,
                    pose_stamped.pose.orientation.y,
                    pose_stamped.pose.orientation.z,
                    pose_stamped.pose.orientation.w
                ])[2],
                "result": result
            })

        rospy.loginfo("=== Navigation Task Completed ===")
        for r in self.goal_results:
            rospy.loginfo("Goal x=%.2f y=%.2f yaw=%.2f : %s",
                          r["x"], r["y"], r["yaw"], r["result"])
        rospy.loginfo("=================================")

        # 标记任务完成
        self.task_done_event.set()

    # 等待任务完成并返回结果
    def wait_for_task_completion(self, timeout=None):
        """
        阻塞等待整个 JSON 任务完成
        :param timeout: 可选超时（秒）
        :return: goal_results 列表
        """
        self.task_done_event.wait(timeout)
        return self.goal_results