# ROS 2 Nav2 多点导航代码解析

单点导航是「去 A」，多点导航是「A → B → C → 回到 A」。Nav2 本身只提供「导航到一个目标」的 action，循环逻辑要自己写——WHEELTEC 的做法是加一个 60 行的 Python 节点，监听 RViz 里点的点，然后一个个喂给 Nav2。

这一篇读两部分：**启动文件怎么把 Nav2 拉起来**，以及**那个循环节点怎么判断「到了」**。

> 源码：
> - `...\2.ROS2机器人应用视频教程：上手使用、雷达与2D导航\8.ROS2 多点导航\wheeltec_nav2.launch.py`（62 行）
> - `nav2_waypoint_cycle.zip` 里的 `nav2_waypoint_cycle/waypoint_cycle.py`（134 行）

## 一、启动文件：三件事

```python
def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    wheeltec_robot_dir = get_package_share_directory('turn_on_wheeltec_robot')
    wheeltec_launch_dir = os.path.join(wheeltec_robot_dir, 'launch')
    wheeltec_nav_dir = get_package_share_directory('wheeltec_nav2')
    wheeltec_nav_launchr = os.path.join(wheeltec_nav_dir, 'launch')

    map_file = LaunchConfiguration('map', default=os.path.join(
        wheeltec_nav_dir, 'map', 'WHEELTEC.yaml'))            # 地图
    param_file = LaunchConfiguration('params', default=os.path.join(
        wheeltec_nav_dir, 'param', 'wheeltec_param', 'param_mini_mec.yaml'))  # 参数

    waypoint_cycle = Node(
        name='waypoint_cycle',
        package='nav2_waypoint_cycle',
        executable='nav2_waypoint_cycle',
    )

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=map_file, ...),
        DeclareLaunchArgument('params', default_value=param_file, ...),
        IncludeLaunchDescription(... 'turn_on_wheeltec_robot.launch.py'),   # 底盘
        IncludeLaunchDescription(... 'wheeltec_lidar.launch.py'),           # 雷达
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([wheeltec_nav_launchr, '/bringup_launch.py']),
            launch_arguments={'map': map_file,
                              'use_sim_time': use_sim_time,
                              'params_file': param_file}.items()),      # Nav2
        waypoint_cycle,                                                     # 循环节点
    ])
```

它组合了三层：

1. **底盘 + EKF**（`turn_on_wheeltec_robot.launch.py`）——提供 `/odom_combined`、TF；
2. **雷达**（`wheeltec_lidar.launch.py`）——提供 `/scan`；
3. **Nav2 bringup**——定位、地图、规划、控制；
4. **自己的循环节点**。

`bringup_launch.py` 是 Nav2 官方 `nav2_bringup` 包里的总启动，它会拉起：`map_server`（读 `WHEELTEC.yaml`）、`amcl`（定位）、`controller_server`（局部控制）、`planner_server`（全局规划）、`behavior_server`（恢复行为）、`bt_navigator`（行为树调度）、`waypoint_follower` 等。

### 1.1 地图与参数是两条独立的路径

```yaml
map:    .../wheeltec_nav2/map/WHEELTEC.yaml
params: .../wheeltec_nav2/param/wheeltec_param/param_mini_mec.yaml
```

`.yaml` 地图文件本身不存图，只存元信息：

```yaml
image: WHEELTEC.pgm        # 相对 yaml 所在目录
resolution: 0.05           # 每像素 5 cm
origin: [-10.0, -10.0, 0.0]
occupied_thresh: 0.65
free_thresh: 0.25
negate: 0
```

`resolution` 和 `origin` 决定了「栅格坐标 → 世界坐标」的换算，写错会让机器人以为自己在地图的另一头。

参数文件则决定规划器和控制器行为（膨胀半径、最大速度、DWB 采样数等）。**地图和参数必须匹配**：用小车参数跑大车地图，或者反过来，轻则路径贴墙，重则原地打转。

### 1.2 `use_sim_time`

```python
use_sim_time = LaunchConfiguration('use_sim_time', default='false')
```

真机为 `false`，用系统时间；Gazebo 仿真必须为 `true`，用 `/clock` 话题。**混用会导致 TF 时间戳对不上**，表现是 `Message Filter dropping message` 或「机器人不动」。

## 二、多点导航节点：状态机

```python
class waypoint_cycle(Node):
    def __init__(self):
        super().__init__('waypoint_cycle')
        self.count = 0      # 一共点了几个点
        self.index = 0      # 当前去第几个
        self.try_again = True

        self.mark_pub = self.create_publisher(MarkerArray, '/path_point', 100)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.create_subscription(PointStamped, '/clicked_point', self.click_callback, 10)
        self.create_subscription(GoalStatusArray,
                                 "navigate_to_pose/_action/status",
                                 self.pose_callback, 10)
```

它只用两种接口：

| 接口 | 类型 | 方向 | 作用 |
|---|---|---|---|
| `/clicked_point` | `PointStamped` 话题 | 订阅 | RViz 的 Publish Point 工具点的位置 |
| `/path_point` | `MarkerArray` 话题 | 发布 | 在 RViz 里画出箭头和编号 |
| `/goal_pose` | `PoseStamped` 话题 | 发布 | 发给 Nav2 的目标 |
| `navigate_to_pose/_action/status` | `GoalStatusArray` 话题 | 订阅 | 监听 action 的状态 |

最后一条是重点：**Nav2 的目标接口是 Action，不是话题**。但 RViz 的「2D Goal Pose」是往 `/goal_pose` 发消息，Nav2 的 `bt_navigator` 订阅它并转成 action 调用。这个节点也走同一条路——发话题最简单，不用写 action client。

而监听结果时，它没有用 action client 的 `get_result()`，而是**直接订阅 action 的隐藏状态话题** `navigate_to_pose/_action/status`。这算一种取巧：能拿到状态，但拿不到「这次 goal 的唯一 ID」，所以无法区分「我发的这个目标」和「别人发的目标」。

### 2.1 点击回调：记录并立即出发

```python
def click_callback(self, msg):
    self.add_marker(msg.point.x, msg.point.y, 0.0, 1.0)          # 箭头
    self.add_marker_number(msg.point.x, msg.point.y, 0.0, 1.0)   # 编号文字
    self.mark_pub.publish(self.markerArray)
    self.mark_pub.publish(self.markerArray_number)
    if self.count == 0:
        self.publish_pose(self.count)     # 第一个点，立刻出发
        self.index += 1
    self.count += 1
```

第一个点立刻发目标，后面的点只是记录。等第一个点到达后，`pose_callback` 再把下一个发出去。

Marker 用的是 `visualization_msgs/Marker`：

- `Marker.ARROW` 画箭头，`scale.x=0.5` 是长度；
- `Marker.TEXT_VIEW_FACING` 画文字，`text=str(self.count)`；
- `header.frame_id = 'map'`——标记必须挂在 `map` 下，否则会跟着机器人动。

`marker.action = Marker.ADD` 表示新增/更新。注意 `id = self.count`，每个点一个 id，RViz 靠 `(ns, id)` 区分。

### 2.2 状态回调：判断「到了没有」

```python
def pose_callback(self, msg):
    if msg.status_list[-1].status != GoalStatus.STATUS_EXECUTING:
        if msg.status_list[-1].status >= GoalStatus.STATUS_SUCCEEDED and self.count > 0:
            self.try_again = True
            if self.index == self.count:       # 到最后一个了
                self.index = 0
                self.publish_pose(self.index)  # 回到第一个，循环
                self.index += 1
            elif self.index < self.count:
                self.publish_pose(self.index)  # 下一个
                self.index += 1
        elif self.count > 0:                   # 没成功
            # 先重试一次，还不行就跳到下一个
            if self.try_again:
                self.publish_pose(self.index - 1)
                self.try_again = False
            elif self.index < len(self.markerArray.markers):
                ...
```

状态常量（`action_msgs/msg/GoalStatus`）：

| 值 | 常量 | 含义 |
|---|---|---|
| 0 | `STATUS_UNKNOWN` | 未知 |
| 1 | `STATUS_ACCEPTED` | 已接受，还没开始执行 |
| 2 | `STATUS_EXECUTING` | 执行中 |
| 3 | `STATUS_CANCELING` | 正在取消 |
| 4 | `STATUS_SUCCEEDED` | 成功 |
| 5 | `STATUS_CANCELED` | 被取消 |
| 6 | `STATUS_ABORTED` | 失败 |

这段逻辑的意图是：**只要不是在执行，就认为本轮结束**；再看状态码是否「大于等于成功」。

但这里有三个需要注意的地方：

1. **`status_list[-1]` 没有判空**。action 服务器在还没有任何目标时也会周期性发布 `GoalStatusArray`，此时 `status_list` 是空列表，`[-1]` 会抛 `IndexError`。实际运行时通常已经有目标，所以不常复现。
2. **`>= STATUS_SUCCEEDED` 把失败也算成功**。`ABORTED = 6`、`CANCELED = 5` 都大于等于 4，都会走「到达」分支。也就是说**导航失败时，车会认为到点了，继续去下一个点**。
3. **`ACCEPTED`、`CANCELING` 也不是 `EXECUTING`**，会进入 `elif` 的「没成功」分支。状态在 1 Hz 左右刷新，理论上存在「刚接受就判失败并重发」的窗口。`try_again` 这个标志位就是为了压制重复发送，但它只在失败分支里复位。

更稳的写法是显式判断终态：

```python
if not msg.status_list:
    return
s = msg.status_list[-1].status
if s == GoalStatus.STATUS_SUCCEEDED:
    advance()
elif s in (GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_CANCELED):
    retry_or_skip()
```

## 三、为什么「多点」不直接用 `NavigateThroughPoses`

Nav2 其实提供了一个专门的多点 action：`navigate_through_poses`，一次传入一串位姿，Nav2 自己按顺序走。它适合「一条路径连续经过多个点」的场景，但**不能循环、不能在每个点停下来做事、失败后不能自定义重试**。

自己写循环节点的价值在于可插入逻辑：

- 到点后开关灯、拍照、播报；
- 某点失败重试 N 次后跳过；
- 按任务列表动态调整顺序；
- 与外部系统（电梯、闸机）对接。

代价是要自己处理状态机、超时、异常——上面那三个问题就是代价。

## 四、运行与验证

```bash
# 启动导航
ros2 launch wheeltec_nav2 wheeltec_nav2.launch.py

# 检查 Nav2 是否就绪
ros2 action list | grep navigate
ros2 node list | grep -E "controller|planner|bt_navigator|amcl"
ros2 topic echo /amcl_pose --once      # 定位是否收敛

# 命令行直接发一个目标（不经过 RViz）
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}}" \
  --feedback
```

RViz 里的操作顺序：

1. 加 `Map` 显示，选 `/map`；
2. 加 `MarkerArray` 显示，选 `/path_point`；
3. 用 `2D Pose Estimate` 给初始位姿（AMCL 需要）；
4. 用 `Publish Point` 依次点目标点，每点一下车就往第一个点走。

## 五、排障顺序

| 现象 | 先查 |
|---|---|
| 车不动，`/cmd_vel` 没数据 | `ros2 action list` 有没有 `navigate_to_pose`；参数文件是否加载 |
| 报 `Timed out waiting for transform` | TF 树断：`odom_combined → base_footprint` 有没有；`map → odom` 有没有 |
| 定位乱跳 | 初始位姿没给；激光和地图对不上；`use_sim_time` 错 |
| 路径贴着墙 | 参数里的 `robot_radius` / `inflation_radius` 与实车不符 |
| 到点不停 | 参数里的 `xy_goal_tolerance` 太小，或代价地图有障碍导致无法到达 |
| 循环节点不继续 | `status_list` 空、状态判断逻辑、`/goal_pose` 有没有被 Nav2 订阅 |

## 六、这段代码的启示

- **Action 是长任务的正确抽象**：导航可能跑几十秒、可能失败、需要反馈，话题做不到这些。Nav2 的核心接口全是 action。
- **话题可以当快捷方式**：`/goal_pose` 让 RViz 和脚本都能发目标，底层再转 action。接口设计上「简单入口 + 完整能力」并存很常见。
- **状态机要显式**：用「不等于执行中」代替「等于成功/失败」，早晚会在边界状态上翻车。
- **判空是免费的鲁棒性**：`status_list[-1]` 前面加三行判断，能省掉一次现场崩溃。

## 相关

- [[SLAM 与导航]] —— Nav2 架构、代价地图、规划器与控制器。
- [[路径规划算法]] —— 全局规划 A*、局部 DWB/TEB。
- [[ROS 2 Launch 文件代码解析]] —— 包含、参数、条件。
- [[ROS 2 TF 与 URDF 代码解析]] —— `map → odom → base_footprint` 三条边。
- [[ROS 2 通信机制]] —— Action 与话题的区别。
- [[WHEELTEC ROS 教程学习报告]]
- [[ROS Index]]
