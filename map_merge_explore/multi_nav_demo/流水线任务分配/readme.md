1. 核心数学模型
A. 代价矩阵与最优匹配 (KM算法)
系统通过构建代价矩阵 $C$，其中 $C_{i,j}$ 表示第 $i$ 台机器人到第 $j$ 个待执行任务的欧几里得距离：
$$C_{i,j} = \sqrt{(x_i - x_j)^2 + (y_i - y_j)^2}$$KM 算法的目标是寻找一个映射 $$f: \text{Robot} \to \text{Task}$$，使得总代价最小：$$\min \sum_{i} C_{i, f(i)}$$
B. 任务预热优化 (Pre-heating)为了减少等待间隙，系统引入了预热系数 $\alpha \in [0, 1]$。当任务 $T_{pre}$ 的剩余距离 $d_{rem} < \text{threshold}$ 或进度超过 $\alpha$ 时，释放后续任务 $T_{next}$ 的锁定，使其进入 KM 分配池。
C. 异常指数退避 (Exponential Backoff)若任务失败次数为 $n$，则该任务重回就绪态前的冻结时间 $T_{wait}$ 为：
$T_{wait} = \text{base\_interval} \times 2^{n-1}$

安装`scipy`库
准备tasks.json文件，
`
[
    {
        "id": 0, 
        "name": "接水", 
        "pose": [1.0, 1.0, 0.0, 0.0], 
        "pre": null
    },
    {
        "id": 1, 
        "name": "磨粉", 
        "pose": [2.5, 1.0, 0.0, 1.57], 
        "pre": 0
    },
    {
        "id": 2, 
        "name": "冲泡", 
        "pose": [4.0, 2.0, 0.0, 3.14], 
        "pre": 1
    },
    {
        "id": 3, 
        "name": "加糖", 
        "pose": [4.0, 3.0, 0.0, 0.0], 
        "pre": 2
    }
]
`
`pose`含义是xyz yaw
pre是前置任务的id 数值越小说明在逻辑链条上越早 null任务会立即进入待分配池

A. KM 算法指派逻辑系统每秒会进行一次“点名”。它会查看当前有哪些机器人是闲着的（idle_robots），有哪些任务是满足条件的（ready_tasks）。然后计算 距离代价矩阵：$Cost_{i,j} = \text{dist}(\text{Robot}_i, \text{Task}_j)$通过 scipy.optimize.linear_sum_assignment 寻找总路径最短的分配方案。
B. 预热优化 (Pre-heating)这是提升效率的关键。代码逻辑如下：机器人 A 正在执行任务 0。调度器实时监控 A 距离任务 0 目标的距离。当 $\text{dist} < 0.5\text{m}$ 时，调度器判定任务 0 “即将完成”。此时，原本依赖任务 0 的任务 1 会被提前释放进入 KM 分配池。机器人 B 此时可能已经领到任务 1 并开始启动，从而消除了 A 停稳到 B 启动之间的延迟。
C. 容错与退避 (Exponential Backoff)如果任务失败（例如路径被堵死），系统不会疯狂重复尝试。失败 1 次：等待 $5 \times 2^0 = 5$ 秒后再分配。失败 2 次：等待 $5 \times 2^1 = 10$ 秒后再分配。以此类推，直到达到 max_retries（3次）后永久放弃该任务并报错。

启动
`python3 your_script.py _robots:="tb3_0,tb3_1" _global_frame:="map" _preheat_dist:=0.3`
注意： 参数前必须加一个下划线 _，这代表设置的是该节点的私有参数（对应代码中的 ~robots）。

代码内部 Key	默认值	说明
~robots	tb3_0,tb3_1,tb3_2	参与任务的机器人 Namespace 列表，用逗号隔开
~global_frame	map	静态全局地图坐标系
~base_frame_suffix	base_footprint	机器人底盘坐标系名称（不含前缀）
~preheat_dist	0.5	数学公式优化项：当前序任务剩余距离小于此值时，触发 KM 预拨分配
~goal_timeout	150.0	单次导航最大容忍时间（秒）
