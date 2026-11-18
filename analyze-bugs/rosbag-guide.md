# ROS 2 bag 离线分析参考

按当前问题选择章节，不要求每次全部执行。话题和消息字段必须从当前录制与源码读取，以下名称均为语义示例。

## 1. 录制清点和来源

- `metadata.yaml` 提供后端、分片、开始/持续时间、话题类型/数量、QoS；以实际文件和消息验证其一致性。
- 用 `ros2 bag info <路径>` 查看不等于回放；首选离线读取，不启动 ROS 节点或发现网络。
- SQLite 使用只读连接；MCAP、压缩文件、rosbag1 需要各自后端。不能按扩展名强行解码。
- 分片合并按**话题名、类型、序列化格式**识别，不假设每个文件的 topic_id 一致；同名异类型必须报告冲突。
- 类型支持需匹配录制消息定义。反序列化失败要保留上下文并停止该类定量结论，不吞掉异常后假称“无数据”。
- 统计总数量及首末消息时间。零消息、订阅后才开始记录、话题录制中途出现，都不能当成节点从未运行。
- QoS 配置、bag 丢帧、采集/传输/录制延迟与显示频率需分别讨论。

## 2. 最小可执行读取例子

准备含 ROS 2 和自定义消息支持的 Python 环境，核对 `sys.executable`；有时是 `/usr/bin/python3`，不要写死所有机器。
以下片段只读取单分片的一个指定话题，不发布任何数据：

```python
import sqlite3
from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

path = Path("/替换为实际分片.db3").resolve()
topic_name = "/替换为实际话题"
db = sqlite3.connect(path.as_uri() + "?mode=ro", uri=True)
row = db.execute(
    "SELECT id, type, serialization_format FROM topics WHERE name=?",
    (topic_name,),
).fetchone()
if row is None:
    raise ValueError(f"未录制话题: {topic_name}")
topic_id, type_name, encoding = row
if encoding != "cdr":
    raise ValueError(f"不支持的序列化格式: {encoding}")
msg_type = get_message(type_name)
origin_ns = db.execute("SELECT MIN(timestamp) FROM messages").fetchone()[0]
for stamp_ns, blob in db.execute(
    "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp,id",
    (topic_id,),
):
    relative_s = (stamp_ns - origin_ns) * 1e-9
    msg = deserialize_message(blob, msg_type)
    # 按当前接口读取小字段；大消息只取关键窗口，不无条件 print(msg)。
db.close()
```

有明确窗口时在 SQL 加 `timestamp>=? AND timestamp<=?`，边界用整数纳秒。
多分片顺序有重叠时按消息时间归并，并保留原分片/行 ID；不要静默去重，重复可能是真实协议重发。

## 3. 应选哪些消息

- 事件身份：目标、Action 状态、任务/控制器命令与状态、`/rosout`。
- 规划：正式轨迹载荷、接受确认、执行进度、局部终点、全局路线；Marker 仅作几何辅助。
- 执行：最终速度/位置/力矩命令，驱动反馈，里程计/IMU，冻结/暂停标志。
- 地图：原始/融合点云、占据与膨胀显示、地图范围、外参和 TF。
- 配置与时间：参数事件、`/clock`、header、节点日志；参数事件不是启动参数的保证快照。

语义审计：Odometry 的 pose 通常在 header frame，twist 通常在 child frame，**实现可能不遵守约定**，必须查生产者。
同时检查 NaN、单位、归一化四元数、header 回退和重启；不要直接把位置差分或 world/body 分量混着比较。

## 4. 速度突变的量化

- 平面速度模长 `sqrt(vx²+vy²)`；矢量跳变 `||v[i+1]-v[i]||`；角速度单独分析。
- 时间间隔用录制/源时间各算一版，并说明采用哪一种。`Δv/Δt` 是离散平均变化率，不能不加说明称作实体加速度。
- 设定并说明绝对阈值、最小有效时间间隔、窗口，检查阈值敏感性；不要只报告排序第一名。
- 不以插值后的曲线代替原始突变值；保留相邻原始消息和纳秒时间。
- 对相邻指令点求对应参考的**完整速度矢量**。只比较模长可能漏掉转向跳变。
- 每个异常关联批次、轨迹接受/STOP、进度及最近数据时间差，切换窗缺进度则单独标“无法可靠配对”。
- 优先列出分类：协议停车、同轨迹执行突变、轨迹衔接跳变、反馈/执行器异常、未知。

## 5. B 样条不是按墙钟直接画点

先查消费者对 knots、degree/order、控制点、起始时间的解释。`order` 有的表示次数，有的表示阶数，不能盲用。
若确认消息是 SciPy 兼容的次数 k 和结点：

```python
import numpy as np
from scipy.interpolate import BSpline

# knots、control_points、degree 均来自已核对的消息语义。
spline = BSpline(knots, control_points, degree, extrapolate=False)
u0, u1 = spline.t[degree], spline.t[-degree - 1]
u = u0 + progress * (u1 - u0)  # 仅当控制器确实按该定义回报 progress
p = spline(u)
v = spline(u, nu=1)
a = spline(u, nu=2)
```

- 核对导数是对样条参数还是物理秒；时间缩放时应用链式法则。
- 与消息 Marker 的起终点/控制器 duration 对照验证；别把控制点折线当 B 样条实际路径。
- 计算长度、偏离、速度/加速度峰值时记录采样间隔；密集采样峰值不是严格连续最大值。需要硬认证时求解析极值或用保守界。
- “最大偏离起终点直连线”与“偏离全局路线”“最小障碍距离”“多余绕行”不同。直连线可能穿墙，不能把长度比直接当优化空间。
- 轨迹衔接单独比较 `p旧(u执行)`/`v旧(u执行)`/`a旧(u执行)` 与 `p新(u0)`/`v新(u0)`/`a新(u0)`。
- 新轨迹使用实测速度而非旧参考时，即使每条内部平滑，跨轨迹参考也可能不连续。
- 控制器进度自适应减速时，几何轨迹导数与沿真实墙钟的参考推进速度可能不同；同实际消费者逐项核对。

## 6. 点云与碰撞

- PointCloud2 按 fields 的 offset/datatype/count、point_step、row_step、width/height 和 is_bigendian 解码。
- 不能对有组织点云忽略每行 padding；检查数据长度，不把所有字段都假定为 little-endian float32 XYZ。
- 核对 frame、TF 与外参、采集时间、机器人自身过滤、地面过滤、高度区间。
- 原始点云可能只含可见表面；显示占据/膨胀点云可能降采样/裁剪/低频发布，与内部碰撞缓存不同。
- 最近邻表面距离只能作为佐证，不能替代概率占据阈值、膨胀栅格、足迹 yaw 和前后偏移的正式碰撞查询。
- 地图外/未知/占据若用不同返回值，逐项保留。非零被统一判失败不等于全段都是实体障碍。
- 读取 Marker 类型再解边界：LINE_LIST 的顶点给出范围，`scale.x` 通常只是线宽；CUBE 才可能用 scale 表尺寸。
- 对候选在地图内、足迹前后检查点在地图内分别验证；不要只看机器人中心。
- 新障碍与告警先后是线索，不是完整因果证明；检查障碍何时进入规划地图与轨迹时空交集。

## 7. 卡顿与恢复循环

- 分别统计指令、实测、状态、原始点云、显示点云的间隔 p50/p95/p99/max、重复时间和长缺口。
- 周期性 STOP/接受/恢复时，量化循环次数、周期、位置净位移与累计路程；两者不可互换。
- 核对恢复预算在哪个事件清零：接受新轨迹、执行一段时间、还是实际脱困？循环很多次不一定违反计数实现。
- 对照规划工作线程、地图锁、执行回调、调度间隔；规划耗时与请求等待、协议确认、机器人停车时长分别计量。
- 区分规划验收和执行安全检查的采样、检查范围、yaw、地图版本、起点时效和执行进度；不要先验认定所有碰撞告警是误报。
- 日志 “动态可行” 可能只验证速度/加速度；是否验证碰撞要追真实实现。

## 8. 缺什么补什么

无法精确归因时建议最小遥测：轨迹/执行 ID、执行时间、参考 p/v/a、实测 p/v/yaw、PID 各项、航向缩放、限幅前后指令、分支标志、有效运行参数和地图查询返回值。
同时给出每项需要解释的假设；不要要求用户无差别无限录制，也不要让补日志本身阻塞高频控制。
