#!/usr/bin/env python3
"""
示例：同进程调用增强版 NavigationManager，获取每个目标和整个任务结果
"""

import rospy
import threading
from nav_task_manager_json_result import NavigationManager
import json

def main():
    rospy.init_node("json_nav_runner_result")

    nav_manager = NavigationManager()

    # 启动 run 循环线程
    threading.Thread(target=nav_manager.run, daemon=True).start()

    # JSON任务
    task_json = """
    {
        "goals": [
            {"x": -2.8, "y": 2.4, "yaw": 0},
            {"x": 1.5, "y": 2.0, "yaw": 1.57},
            {"x": 2.0, "y": -1.5, "yaw": 3.14}
        ]
    }
    """

    nav_manager.add_json_task(task_json)
    rospy.loginfo("JSON task added. Robot will start navigation.")

    # 阻塞等待任务完成
    results = nav_manager.wait_for_task_completion()
    rospy.loginfo("Task finished. Results:")
    for r in results:
        rospy.loginfo("Goal x=%.2f y=%.2f yaw=%.2f : %s",
                      r["x"], r["y"], r["yaw"], r["result"])

if __name__ == "__main__":
    main()