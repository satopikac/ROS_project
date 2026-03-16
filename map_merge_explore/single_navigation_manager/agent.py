#!/usr/bin/env python3
import os
import json
import rospy
import tf
from geometry_msgs.msg import PoseStamped

# 导入你提供的 NavigationManager 模块
from nav_task_manager_json_result import NavigationManager

# 使用 OpenAI SDK 调用 DeepSeek
from openai import OpenAI

# -----------------------------
# 配置 DeepSeek API
# -----------------------------
DEEPSEEK_API_KEY = "sk-a5a3732b4085403989f03447e67444cf"
client = OpenAI(api_key=DEEPSEEK_API_KEY, base_url="https://api.deepseek.com/v1")

# -----------------------------
# 加载语义地图
# -----------------------------
with open("semantic_map.json", "r", encoding="utf-8") as f:
    semantic_map = json.load(f)

# -----------------------------
# 使用 DeepSeek 解析自然语言指令，输出语义路径
# -----------------------------
def plan_semantic_path(instruction: str, objects_json: dict):
    prompt = f"""
你是一个机器人导航规划师。
你只能根据以下语义地图上的物体进行路径规划：
{json.dumps(objects_json, ensure_ascii=False, indent=2)}

任务指令是："{instruction}"
请输出访问顺序，只包含地图中存在的物体名称，
并以 JSON 列表格式回复，例如：
["咖啡机", "沙发"]
"""
    print(prompt)
    response = client.chat.completions.create(
        model="deepseek-chat",
        messages=[
            {"role": "system", "content": "你是导航规划专家"},
            {"role": "user", "content": prompt}
        ],
        temperature=0
    )

    text = response.choices[0].message.content.strip()
    try:
        path = json.loads(text)
    except Exception as e:
        raise RuntimeError(f"DeepSeek 输出无法解析为 JSON 列表: {text}") from e

    return path

# -----------------------------
# 将语义名称转换为 JSON 任务点
# -----------------------------
def semantic_path_to_json_goals(path):
    goals = []
    for name in path:
        found = False
        for obj in semantic_map["objects"]:
            if obj["name"] == name:
                goals.append({"x": obj["x"], "y": obj["y"], "yaw": obj["yaw"]})
                found = True
                break
        if not found:
            raise KeyError(f"语义地图中找不到物体: {name}")
    return {"goals": goals}

# -----------------------------
# 主程序
# -----------------------------
def main():
    rospy.init_node("semantic_deepseek_nav_agent")
    nav_manager = NavigationManager()

    instruction = input("请输入任务指令，例如“泡咖啡送到沙发”：")

    # 调用 DeepSeek 获取语义路径
    try:
        semantic_path = plan_semantic_path(instruction, semantic_map)
        print("规划语义路径:", semantic_path)
    except Exception as e:
        print("规划失败:", e)
        return

    # 转换为 JSON 任务，并加入 NavigationManager
    json_task = semantic_path_to_json_goals(semantic_path)
    nav_manager.add_json_task(json.dumps(json_task))

    print("任务已添加到 NavigationManager，开始执行...")
    nav_manager.run()  # 阻塞运行，支持 s/q 跳过/停止

if __name__ == "__main__":
    main()