#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
import threading
import time
import numpy as np
from scipy.optimize import linear_sum_assignment

import rospy
import actionlib
import tf2_ros
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from actionlib_msgs.msg import GoalStatus

class TaskStatus:
    PENDING = 0   # 等待中
    READY = 1     # 依赖已满足，可被KM分配
    ACTIVE = 2    # 执行中
    SUCCEEDED = 3 # 已完成
    FAILED = 4    # 故障

class KMSequentialOptimized:
    def __init__(self, json_path):
        rospy.init_node("km_optimized_nav")
        
        # --- 基础参数 ---
        self.robots = [r.strip() for r in rospy.get_param("~robots", "tb3_0,tb3_1,tb3_2").split(",")]
        self.global_frame = rospy.get_param("~global_frame", "map")
        self.base_frame_suffix = rospy.get_param("~base_frame_suffix", "base_footprint")
        
        # --- 优化参数 ---
        self.preheat_dist = 0.5  # 预热距离阈值：当前序任务距离目标小于0.5m时，释放后续任务
        self.max_retries = 3     # 单个任务最大尝试次数
        self.goal_timeout = rospy.Duration(150.0)

        # --- 核心组件 ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        
        # --- 任务管理 ---
        self.task_pool = self._load_tasks(json_path)
        self.active_assignments = {} # {robot_name: task_id}
        self.fail_counts = {}        # {task_id: count}
        self.backoff_until = {}      # {task_id: timestamp}

    def _load_tasks(self, path):
        with open(path, 'r') as f:
            data = json.load(f)
            for item in data:
                item.update({'status': TaskStatus.PENDING, 'robot': None, 'current_dist': 999.0})
                self.fail_counts[item['id']] = 0
                self.backoff_until[item['id']] = 0
            return data

    def get_pose(self, frame_id):
        """通用坐标获取接口"""
        try:
            trans = self.tf_buffer.lookup_transform(self.global_frame, frame_id, rospy.Time(0), rospy.Duration(0.2))
            return (trans.transform.translation.x, trans.transform.translation.y)
        except: return None

    def _check_preheat(self, pre_id):
        """检查前序任务是否触发预热逻辑"""
        if pre_id is None: return True
        for t in self.task_pool:
            if t['id'] == pre_id:
                # 若已完成，或距离目标极近（预热），则允许后续任务开始分配
                if t['status'] == TaskStatus.SUCCEEDED: return True
                if t['status'] == TaskStatus.ACTIVE and t['current_dist'] < self.preheat_dist:
                    return True
        return False

    def scheduler_loop(self):
        """主调度逻辑：KM指派 + 状态巡检"""
        rate = rospy.Rate(1) # 1Hz 调度频率
        while not rospy.is_shutdown():
            now = time.time()
            with self.lock:
                # 1. 任务状态更新：检查谁满足 READY 条件
                ready_tasks = []
                for t in self.task_pool:
                    if t['status'] in [TaskStatus.PENDING, TaskStatus.FAILED]:
                        # 异常退避检查
                        if now < self.backoff_until[t['id']]: continue
                        # 尝试次数检查
                        if self.fail_counts[t['id']] >= self.max_retries: continue
                        
                        if self._check_preheat(t['pre']):
                            ready_tasks.append(t)

                # 2. 机器人状态更新：谁是空闲的
                idle_robots = [r for r in self.robots if r not in self.active_assignments]

                # 3. 执行 KM 算法
                if ready_tasks and idle_robots:
                    n_r, n_t = len(idle_robots), len(ready_tasks)
                    cost_mat = np.zeros((n_r, n_t))
                    r_poses = [self.get_pose(f"{r}/{self.base_frame_suffix}") for r in idle_robots]

                    for i in range(n_r):
                        for j in range(n_t):
                            if r_poses[i] is None: cost_mat[i,j] = 1000.0
                            else:
                                tp = ready_tasks[j]['pose']
                                cost_mat[i,j] = math.hypot(r_poses[i][0]-tp[0], r_poses[i][1]-tp[1])

                    r_idx, t_idx = linear_sum_assignment(cost_mat)
                    for ri, ti in zip(r_idx, t_idx):
                        robot, task = idle_robots[ri], ready_tasks[ti]
                        task['status'] = TaskStatus.ACTIVE
                        self.active_assignments[robot] = task['id']
                        threading.Thread(target=self.execute_task_wrapper, args=(robot, task)).start()

            if all(t['status'] == TaskStatus.SUCCEEDED for t in self.task_pool):
                rospy.loginfo("MISSION ACCOMPLISHED"); break
            rate.sleep()

    def execute_task_wrapper(self, robot, task):
        """带实时距离反馈和异常退避的执行器"""
        client = actionlib.SimpleActionClient(f"/{robot}/move_base", MoveBaseAction)
        if not client.wait_for_server(rospy.Duration(5.0)): return

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.global_frame
        goal.target_pose.pose.position.x = task['pose'][0]
        goal.target_pose.pose.position.y = task['pose'][1]
        q = quaternion_from_euler(0, 0, task['pose'][3])
        goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y, \
        goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w = q
        
        client.send_goal(goal)

        # 执行期间实时更新距离，供预热逻辑参考
        while not self.stop_event.is_set():
            state = client.get_state()
            if state not in [GoalStatus.PENDING, GoalStatus.ACTIVE]: break
            
            r_pose = self.get_pose(f"{robot}/{self.base_frame_suffix}")
            if r_pose:
                task['current_dist'] = math.hypot(r_pose[0]-task['pose'][0], r_pose[1]-task['pose'][1])
            time.sleep(0.5)

        client.wait_for_result()
        final_state = client.get_state()

        with self.lock:
            if final_state == GoalStatus.SUCCEEDED:
                task['status'] = TaskStatus.SUCCEEDED
                rospy.loginfo(f"[{robot}] Success: Task {task['id']}")
            else:
                # 异常处理：增加失败计数并设置指数退避时间
                self.fail_counts[task['id']] += 1
                wait_time = 5 * (2 ** (self.fail_counts[task['id']] - 1))
                self.backoff_until[task['id']] = time.time() + wait_time
                task['status'] = TaskStatus.FAILED
                rospy.logerr(f"[{robot}] Failed task {task['id']}. Backoff: {wait_time}s")
            
            if robot in self.active_assignments: del self.active_assignments[robot]

    def run(self):
        self.scheduler_loop()

if __name__ == "__main__":
    import os
    path = os.path.join(os.path.dirname(__file__), "tasks.json")
    KMSequentialOptimized(path).run()
