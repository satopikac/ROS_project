# ROS 键盘输入发布器代码解析

key_publish.py该脚本是一个简单的 **ROS 节点**，用于监听键盘输入，并将特定按键封装为 `std_msgs/String` 类型的消息发布到 `/keyboard_input` 话题上。
## 📌 功能概述

- 监听用户在终端中的按键输入（非阻塞）
- 将识别出的有效按键（`c`, `s`, `l`, `+`, `-`）发布到 `/keyboard_input` 主题
- 支持周期性运行（10Hz）

适用于控制机器人运动模式或调整参数的场景，例如：
- `c`: 圆形轨迹
- `s`: 正弦轨迹
- `l`: 直线运动
- `+`: 加速
- `-`: 减速

---

## 🧠 模块与依赖

```python
import rospy
from std_msgs.msg import String
import sys
import select
import termios
```

| 模块 | 功能 |
|------|------|
| `rospy` | ROS 的 Python 客户端库 |
| `String` | ROS 中标准字符串消息类型 |
| `sys` | 获取标准输入文件描述符 |
| `select` | 实现非阻塞输入检测 |
| `termios` | 控制终端属性（实现单字符读取） |

---

## 🔍 KeyPoller 类详解

这是一个上下文管理器类，用于设置终端为“非规范模式”，从而支持单个按键读取。

### ✅ `__enter__()` 方法

- 获取终端的标准输入文件描述符（`stdin`）
- 设置终端为非缓冲模式（关闭回显和规范输入处理）

```python
self.new[3] = self.new[3] & ~termios.ICANON & ~termios.ECHO
termios.tcsetattr(self.fd, termios.TCSAFLUSH, self.new)
```

### ✅ `poll(timeout=0.1)` 方法

- 使用 `select.select()` 判断是否有输入可用
- 如果有，读取一个字符；否则返回 `None`

```python
dr, dw, de = select.select([sys.stdin], [], [], timeout)
if not dr:
    return None
return sys.stdin.read(1)
```

### ✅ `__exit__()` 方法

- 程序退出时恢复终端原始设置

```python
termios.tcsetattr(self.fd, termios.TCSAFLUSH, self.old)
```

---

## 🚀 key_publisher() 函数详解

### 初始化 ROS 节点

```python
rospy.init_node('key_publisher', anonymous=True)
pub = rospy.Publisher('/keyboard_input', String, queue_size=10)
rate = rospy.Rate(10)  # 10Hz
```

- 创建名为 `key_publisher` 的节点
- 发布者对象 `pub` 向 `/keyboard_input` 发布 `String` 类型数据
- 设置循环频率为 10Hz

### 日志提示信息

```python
rospy.loginfo("Use keys: [c] Circle, [s] Sinusoid, [l] Straight Line, [+] Speed Up, [-] Speed Down")
```

### 主循环

使用 `with KeyPoller()` 进入上下文管理器，确保终端设置正确。

```python
while not rospy.is_shutdown():
    key = key_poller.poll()
    if key:
        if key in ['c', 's', 'l', '+', '-']:
            msg = String()
            msg.data = key
            pub.publish(msg)
    rate.sleep()
```

- 每次循环尝试读取按键
- 若按键有效，则构造并发布消息
- 保持 10Hz 循环频率

---


## 📎 总结

| 特性 | 描述 |
|------|------|
| ROS 节点名称 | `key_publisher` |
| 发布主题 | `/keyboard_input` |
| 消息类型 | `std_msgs/String` |
| 支持按键 | `c`, `s`, `l`, `+`, `-` |
| 更新频率 | 10Hz |
| 终端输入方式 | 非阻塞、单字符读取 |

---


