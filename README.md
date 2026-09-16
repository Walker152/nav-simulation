# Sentry simulation

这个模块将 Gazebo Fortress 仿真资源收敛到当前仓库，并直接接入现有导航主链路：

```text
Gazebo 左右双 MID360（每颗由前/后 180° GPU LiDAR 拼接）+ 水平 IMU
  -> ros_gz_bridge
  -> 双雷达公共帧融合 + MID360 pattern adapter
     (/livox/lidar: livox_ros_driver2/msg/CustomMsg)
  -> Point-LIO (/aft_mapped_to_init, /cloud_registered_full)
  -> model PCD GICP (pcd_map -> camera_init, 2025/2026 场地)
  -> ROGMap + MINCO planner
  -> Minco MPC controller (/cmd_vel_mpc, base 坐标系)
  -> sentry_sim_cmd_adapter
  -> Gazebo omni/diff chassis
```

仿真不会启动 `communication`，也不会使用真车裁判系统或下位机链路。Gazebo 的
`/sim/ground_truth/odom` 只用于验真，不会替代 Point-LIO 里程计作为导航输入。

## 功能和边界

| 功能 | 实现 | 说明 |
|---|---|---|
| 全向底盘 | `sentry_omni` + Fortress `MecanumDrive` | 接收转换后的车体系速度，可测试 X/Y/yaw |
| 差速底盘 | `sentry_diff` + Fortress `DiffDrive` | 模型保留；当前导航仅支持 omni，差速导航显式拒绝 |
| 双 MID360 | 左右镜像雷达 + pattern adapter | 合并为 `/livox/lidar`，消息类型与真机相同 |
| IMU | Gazebo IMU + 仿真滤波器 | 200 Hz，输出 `/sim/imu` 给 Point-LIO |
| 里程计 | Point-LIO | 导航使用 `/aft_mapped_to_init`，不是 Gazebo 真值 |
| 重定位 | 一次性 ratio-GICP | 2025/2026 场地按 `worlds.yaml` 加载模型 PCD |
| 导航闭环 | Nav2 + ROGMap + MINCO + MPC | RViz 下发目标后规划并驱动车辆 |
| 真值验收 | Gazebo odometry | `/sim/ground_truth/odom` 只用于比较误差 |

本模块不会启动 `communication`、裁判系统或真实下位机。修改仿真模型、bridge 和仿真参数不会改变实车启动链路。

## 已包含的资源

- RMUC / RMUL 2024、2025 以及 RMUC 2026 场地 SDF、网格与二维地图。
- RMUC 2026 当前放置了点云重建的临时视觉/碰撞 STL。视觉模型存在较多重建坑洞，
  仅用于资源接入和坐标检查，等待外部 CAD/网格工具生成的修复版本覆盖；不要把当前
  RMUC 2026 网格视作已完成的坡道、碰撞和导航验收结果。
- `sentry_omni`：轮心按 `0.44 m × 0.44 m` 正方形布置的四轮全向底盘，使用
  包内构建的 `MecanumDrive2`，PB2025 chassis/gimbal 网格仅作视觉模型。
- `sentry_diff`：半径 `0.3 m` 的圆形两轮差速底盘，仅左右两轮驱动，前后球形
  万向支承，使用 Fortress 原生 `DiffDrive`。
- 双 MID360：左右镜像安装，位姿以模型 SDF 和 launch 的测量帧转换为准；每颗水平 360°、垂直
  -7.3°～52.3°、0.1～40 m、10 Hz；按 80 万条真实非重复扫描模式轮转采样。
  每颗每帧最多 20,000 个 `CustomPoint`，融合后目标点率 400,000 points/s。
- RMUC 使用 ICP 包内非 red/blue 的模型 PCD；RMUL 使用 Nav2 地图目录中的模型 PCD。
  两份资源复制到仿真包，避免运行时依赖源码工作目录。
- 200 Hz IMU、Point-LIO `CustomMsg` 输入参数和完整导航参数。
- 当前仓库 Nav2、ROGMap、MINCO、MPC 的仿真参数副本。

第三方资源来源和许可见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。

## 环境与构建

目标环境是 Ubuntu 22.04、ROS 2 Humble、Gazebo Fortress（Ignition Gazebo 6）。需要
`ros_gz_sim`、`ros_gz_bridge`、Ignition Gazebo 6 开发包，以及本仓库现有依赖。

依赖安装示例（请先按本机软件源核对包名）：

```bash
sudo apt install ros-humble-ros-gz ignition-fortress libignition-gazebo6-dev
```

首次使用先检查 ROS 和 Gazebo：

```bash
source /opt/ros/humble/setup.bash
ros2 pkg prefix ros_gz_sim
ros2 pkg prefix ros_gz_bridge
ign gazebo --versions
```

在仓库根目录构建示例：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select sentry_simulation
```

启动脚本会检查 `sentry_simulation`、ROS–Gazebo bridge、`point_lio`、
`icp_relocalization`、`navi2`、
`minco_planner`、`minco_controller` 和 `rog_map`。全向底盘使用随包构建和安装的
`MecanumDrive2`，仿真适配器直接转发车体系速度；实车通信节点不参与这条链路。

## 快速启动

以下命令在 ROS 工作空间根目录执行。先运行依赖预检，不启动 Gazebo：

```bash
cd /home/alioth/nature_will
./src/scripts/simlation.bash omni rmuc_2025 --check
```

脚本位置参数和选项：

| 参数 | 可选值 | 默认值 | 作用 |
|---|---|---|---|
| 第 1 个位置参数 | `omni`、`ackermann`、`diff` | `omni` | 默认统一 YAML 的底盘选择；diff 资源保留但导航拒绝 |
| 第 2 个位置参数 | `rmuc_2024`、`rmul_2024`、`rmuc_2025`、`rmuc_2026`、`rmul_2025` | `rmuc_2025` | 选择场地 |
| `--params-file PATH` | 任意统一导航 YAML | 空 | 显式参数文件；此时第 1 个参数必须与 `planner.model` 一致 |
| `--headless` | 开关 | 关闭 | 不启动 Gazebo GUI，只运行 server |
| `--no-rviz` | 开关 | 关闭 | 不启动 RViz |
| `--check` | 开关 | 关闭 | 只检查 ROS overlay 和依赖包 |

RViz 随 launch 直接启动，导航初始化期间也能查看地图和状态。等终端出现
`Managed nodes are active` 后再下发导航目标；窗口出现不代表导航已经就绪。
`--no-rviz` 继续用于完全关闭 RViz。

常用启动方式：

```bash
# 全向底盘 + RMUC 2025（默认）
./src/scripts/simlation.bash omni rmuc_2025

# 阿克曼底盘 + RMUC 2024（使用统一 YAML 的运行时模型选择）
./src/scripts/simlation.bash ackermann rmuc_2024 --headless --no-rviz

# 全向底盘 + RMUC 2026 点云重建场地
./src/scripts/simlation.bash omni rmuc_2026

# 差速底盘 + RMUL 2025
# 差速导航尚未实现，不使用 diff 启动闭环导航。

# 无 Gazebo GUI，也不启动 RViz
./src/scripts/simlation.bash omni rmuc_2024 --headless --no-rviz
```

可选 world：`rmuc_2024`、`rmul_2024`、`rmuc_2025`、`rmuc_2026`、`rmul_2025`。
2025 与 RMUC 2026 场地默认启用模型 PCD GICP；2024 场地因现有 PCD 与旧场地几何不一致，保持
实车初值静态定位，不执行可能误收敛的 ICP。所有随包 PGM YAML 原点均为 `[0, 0, 0]`。
RMUL 2025 使用与模型 PCD 同源的 2026 RMUL PGM，并把居中的场地网格平移到零原点地图。

需要临时关闭 GICP、改变日志级别或直接使用 launch 参数时，先完成构建并 source 工作空间：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch sentry_simulation simulation.launch.py \
  chassis_type:=omni world:=rmuc_2025 \
  headless:=false rviz:=true use_icp:=false log_level:=info
```

直接 `ros2 launch` 使用 install tree；新增或替换资源后应重新执行 symlink-install 构建。`src/scripts/simlation.bash` 会额外指定源码侧 simulation share 和模型搜索路径，适合当前仓库日常启动。

### 导航参数与对照验证

Ackermann 仿真使用 `AckermannBicycle` 单一底盘命令插件：同一曲率同时设置左右前轮角和后轮差速，
`steering_limit=0.4` 表示自行车中心转角。前轮转向位置直接重置并保持，滚动关节保持自由；这有意
取消旧内置插件约 1 秒的转向伺服，不用于模拟实车有限转向响应。车身惯性、轮胎接触、碰撞、传感器
及原 groundtruth/关节反馈仍由 Gazebo 物理系统产生；插件不强制车身位姿，也不自行发布 odom/TF。
零速纯 yaw 请求停车回正，非有限输入也停车回正。原命令 adapter 继续负责超时停车。

修改插件后先构建并 source 对应 install，使 Gazebo 能找到插件；可独立运行真实物理回归：

```bash
python3 src/simulation/sentry_simulation/test/ackermann_physics_regression.py --output /tmp/ack_physics_check
```

回归自建唯一 IGN 分区和无传感器空地 world，保留原模型全部物理实体，结束只清理自身进程；
输出真实轮角、后轮速度、groundtruth CSV 和断言摘要。`--smoke` 仅检查直行通信。
`--nonfinite` 单独从运动状态测试 NaN/Inf 输入停车，只用于新插件，不对旧插件执行。
测试检查转向和实际车身响应，轮角到位并不单独证明导航闭环成功。
正倒车之间先停车稳定；直接从 +0.6 跳到 −0.6 m/s 且同时回正的极端输入曾产生约 200 ms
接触瞬态，未通过 150 ms 车身门。该限制保留，测试没有通过强制车身状态或放宽响应门消除它。

默认读取 `navi2/params/navigation.yaml`，按
`frames / odometry / planner / controller / omni / ackermann` 分组；脚本的
`omni` 或 `ackermann` 参数选择对应模型，Nav2 宿主和
`use_sim_time=true` 由 launch 统一装配。保留 20 Hz 控制、0.05 s MPC 步长、
Q/R、车辆能力、ROG 更新周期和 footprint。共享测量原点为
`sensor_in_base.xyz: [0, -0.2, 0]`，匹配 Point-LIO 当前平面导航 base。

复制这份 YAML 调整权重或能力后，可显式选择文件，路径含空格时使用引号：

```bash
ros2 launch sentry_simulation simulation.launch.py \
  world:=rmuc_2024 use_icp:=false headless:=true rviz:=true \
  params_file:=/绝对路径/nav2_sim_compare.yaml
```

对照文件保留仿真输入链的 frame、topic 和外参。直接传递原文件路径，
其中相对地图路径按该 YAML 所在目录解析。当前样例由场地选择加载全局地图，
ROGMap 使用在线点云，prior_map 保持关闭。

只验证参数、仿真时钟和生命周期时，可使用测试输入，完全不启动 Gazebo：

```bash
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=195 MINCO_RUN_LAUNCH_TESTS=1 \
  /usr/bin/python3 -m pytest src/navigation/navi2_bringup/test/test_navigation_lifecycle.py \
  -q -k nav2_sim
```

这六项检查覆盖独立、混合组合和外部 LiDAR 容器入口，核验实际 `use_sim_time`、
planner/controller 外参以及 costmap 实际发布的轮廓；分别执行 lifecycle shutdown
和持续点云更新后的 SIGINT，要求所有子进程正常退出。它们不替代 Gazebo 中的运动闭环验证。

### 仿真点云传输

Point-LIO 的完整点云 publisher 使用 `UniquePtr`，ROGMap cloud subscription 显式启用
intra-process。ROGMap 的四类互斥回调由同一进程内的专属四线程 executor 调度，
退出先停止并 join executor，再销毁回调组，避免 Humble 等待队列访问已释放的 guard condition。
该调整不更改点云话题、消息格式、QoS 或频率。实车容器也需要预加载同一份参数，
步骤见[同进程通信与退出知识文档](../docs/knowledge/20260908_navigation_composition_shutdown.md)。

`simulation.launch.py` 默认给本次仿真进程设置 `FASTRTPS_DEFAULT_PROFILES_FILE`，
加载 `sentry_simulation/config/fastdds_shm.xml`。该配置为每个 Fast DDS participant
提供 16 MiB 的共享内存段，并保留 UDPv4 传输。四路 GPU LiDAR 的单条原始点云约 1.38 MB，超过
Fast DDS 原有 512 KiB 共享内存段；扩大容量避免大消息发送失败造成点云缺帧和四路
同步等待。配置不改变点云 QoS、同步容差、传感器频率或实车启动链路。

环境变量只在仿真 launch 的作用域内生效，不会覆盖同一父 launch 中其他组件的环境。
如果启动环境已经设置了非空 `FASTRTPS_DEFAULT_PROFILES_FILE`，仿真沿用该配置；
使用其他 RMW 实现时该 Fast DDS 配置不生效。若设置了 `ROS_LOCALHOST_ONLY=1`，
仿真不会自动加载此配置，以保留已有的本机通信限制；此时如需扩大 SHM，应由用户
提供同时保留本机限制的 DDS profile。

## 导航使用方法

完整闭环的推荐检查顺序：

1. 启动仿真，等待 Gazebo、Point-LIO、Nav2、ROGMap，以及场地启用时的 GICP 初始化。
2. 在 RViz 确认二维地图、点云、机器人位姿和障碍投影方向一致。
3. 观察静止状态下里程计的 Z、roll、pitch，确认没有持续漂移或向下发散。
4. 使用 RViz 的 `2D Goal Pose` 发布目标。
5. 确认 `/cmd_vel_mpc` 有输出，车辆在 Gazebo 中移动且 `/aft_mapped_to_init` 连续更新。
6. 分别测试平地、中央区域、坡道、墙角和窄通道；对照 `/sim/ground_truth/odom` 判断定位误差。

核心话题：

| 话题 | 类型 | 用途 |
|---|---|---|
| `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` | 双 MID360 融合后的 Point-LIO 输入 |
| `/sim/imu` | `sensor_msgs/msg/Imu` | 仿真 IMU 滤波输出 |
| `/aft_mapped_to_init` | `nav_msgs/msg/Odometry` | Point-LIO 里程计和导航状态 |
| `/cloud_registered_full` | `sensor_msgs/msg/PointCloud2` | ROGMap 与 GICP 使用的完整点云 |
| `/cmd_vel_mpc` | `geometry_msgs/msg/Twist` | MPC 输出的车体系速度 |
| `/sim/cmd_vel` | `geometry_msgs/msg/Twist` | 转换后发送给 Gazebo 底盘的车体系速度 |
| `/sim/ground_truth/odom` | `nav_msgs/msg/Odometry` | Gazebo 真值，仅用于验收 |

启动后可依次检查频率和数据：

```bash
ros2 topic hz /livox/lidar
ros2 topic info /livox/lidar
ros2 topic hz /sim/imu
ros2 topic hz /aft_mapped_to_init
ros2 topic hz /cloud_registered_full
ros2 topic hz /cmd_vel_mpc
ros2 topic echo /sim/ground_truth/odom --once
```

如果 `/cmd_vel_mpc` 有输出但车辆不动，再检查 `/sim/cmd_vel` 和
适配器的超时状态；MPC 已输出车体系速度，适配器直接转发，不依赖 odom 做第二次旋转。

## 坐标、时间与速度语义

- 全部节点启用 `/clock` 和 `use_sim_time`。
- `config/nav2_sim.yaml` 使用与实车相同的机器人配置分组，由 navi2 launch 合并宿主参数。planner 与 Point-LIO、点云适配器保持在 `livox_pointlio_container` 同一进程。启动容器前将机器人配置与 `nav2_host.yaml` 合成为进程级 ROS 参数，使 planner 内部创建的 global costmap 也能继承完整配置；退出时删除本次临时参数文件。
- 融合 LiDAR/IMU link 在模型中的位姿为 `(0,-0.2,0.3,0,0,0)`，
  物理 base_link 位于 `(0,0,0.2,0,0,0)`。适配器将左右测量点转换到该 IMU 公共帧，
  不会把里程计原点移到车体中心。
- 2025/2026 场地通过 `map -> pcd_map -> camera_init` 接入一次性 GICP；超时会回退到同一
  实车初始位姿。2024 场地直接发布 `map -> camera_init`。两种路径的定位原点 Z 都为零。
- planner/controller 共用 `[0,-0.2,0]` 平面外参，位置及旋转引起的杆臂速度一起修正。
  Z 为零是沿用 Point-LIO 导航 base 与 body 等高的约定；与 Gazebo 物理 link 的三维外参不同。
- Nav2 使用同时包络全向轮和差速轮的凸多边形 footprint；MINCO 优化安全距离为
  `0.45 m`，避免原先 `0.20/0.25 m` 低估车体后卡场地边角。
- Point-LIO 发布 `camera_init -> body` 和 `camera_init -> base_link`；Odometry 为
  `camera_init / body`。当前 base TF 仅保留 yaw，三维坡面 TF 的统一不在这次参数修正范围。
- MPC 在 odom 求解，`/cmd_vel_mpc` 为车体系速度；适配器直接转发。
- 当前只允许 omni 导航；差速模型保留为仿真资源，需规划与控制成对实现后再接入。
- MPC 命令超过 0.25 s 未刷新时，适配节点会向 Gazebo 发送一次零速度，避免底盘保持旧指令。
- `/livox/lidar` 与真机相同使用 `livox_ros_driver2/msg/CustomMsg`：`offset_time`
  单位为纳秒，`line` 为 0～3，`tag` 为 `0x10`，所以 Point-LIO 配置为
  `lidar_type: 1`、`scan_line: 4`、`timestamp_unit: 3`。

## STEP / CAD 资源是否需要

跑通定位、规划和控制闭环不需要 STEP：当前视觉模型来自 PB2025 描述链路，物理层则
刻意采用独立的低复杂度碰撞体、真实质量/惯量、轮距、轮径和传感器位姿，避免把高面数
渲染网格直接作为 collision 导致接触不稳定。

如果要研究翻坡、台阶通过性、碰撞边界、重心转移或做 2026 实车等比例验证，则建议补充：

1. 底盘和上装 STEP（最好拆分可动件），以及实际总质量、重心和惯量。
2. 轮径、左右轮距、前后轴距、离地间隙和悬挂/轮胎等效参数。
3. LiDAR、IMU 相对底盘中心的精确 XYZ/RPY 外参。
4. 场地新版本 CAD、材质摩擦系数和关键坡面/台阶尺寸。

STEP 不能直接作为高频物理碰撞网格使用。推荐保留 STEP 作为尺寸母版，导出 DAE/STL
作视觉网格，再单独制作低面数、凸分解后的 collision；随后把真实质量、重心和惯量填写到
两个 `model.sdf`。不要把整车高面数网格直接用于 collision，否则会显著降低实时性并产生
不稳定接触。

## 场地资源布局与替换

每个场地由四类资源组成：

```text
sentry_simulation/
├── config/worlds.yaml                    # 场地注册、出生点、GICP 配置
├── maps/<world>.pgm                      # Nav2 二维地图
├── maps/<world>.yaml
├── resource/maps/pcd/*.pcd               # worlds.yaml 指定的 GICP 目标点云
├── resource/models/<world>/
│   ├── model.config
│   ├── model.sdf
│   └── meshes/                            # visual/collision STL 或 DAE
└── resource/worlds/<world>_world.sdf     # Gazebo world 入口
```

替换 RMUC 2026 外部重建结果时，保持单位为米、Z-up、地图原点和 XYZ/RPY 不变，直接覆盖：

```text
sentry_simulation/resource/models/rmuc_2026/meshes/rmuc_2026_visual.stl
sentry_simulation/resource/models/rmuc_2026/meshes/rmuc_2026_collision.stl
```

视觉和碰撞 STL 必须使用完全相同的坐标系。替换后至少检查文件边界、三角面数量、孔洞/非流形面、出生点地面高度、坡道与台阶尺寸，再运行全向/差速车辆通过性验收。不要自动居中模型，也不要在 `model.sdf` 中用未知平移补偿错误坐标。

新增其他场地时，复制现有同类型目录并在 `config/worlds.yaml` 注册；随后同步更新`src/scripts/simlation.bash` 的 world 白名单。场地名称、目录名、model URI 和 catalog key 应保持一致。

## 常见问题

### `Unsupported world` 或 `unknown world`

前者来自根脚本白名单，后者来自 `config/worlds.yaml`。确认两个位置都注册了同一个场地名，并检查 world/map 文件路径。

### Gazebo 找不到 `model://...`

优先使用`src/scripts/simlation.bash`，它会设置 `IGN_GAZEBO_RESOURCE_PATH`、`GZ_SIM_RESOURCE_PATH`、`SDF_PATH` 和 `IGN_FILE_PATH`。直接 launch 时需要重新构建并 source 最新 install tree。

### 有导航目标，但小车不动

依次检查 `/aft_mapped_to_init`、`/cmd_vel_mpc`、`/sim/cmd_vel`。没有新鲜里程计时 MPC 输出零；只有 `/cmd_vel_mpc` 没有 `/sim/cmd_vel` 时，重点检查适配器进程与订阅；两者都有但 Gazebo 不动时，再检查底盘 plugin 和实体名称。

### 里程计 Z 发散或 ROGMap 把地面投成障碍

先比较 `/aft_mapped_to_init` 与 `/sim/ground_truth/odom`，再检查 IMU、双雷达时间戳、模型碰撞面和平地高度。碰撞网格的坑洞、假坡和薄板塌缩会直接造成车体姿态抖动，不能只通过 Point-LIO 参数掩盖。

### GICP 没有接受结果

确认目标 PCD 与 world 是同一场地、单位和坐标方向，检查 `worlds.yaml` 中的 `pcd_to_map`、`min_inlier_ratio` 和 `min_overlap_ratio`。调低比例阈值之前先查看实际重叠和几何一致性；超时回退只保证初值存在，不代表模型点云已经正确对齐。

## 当前验证边界

模块提供静态契约、Python 运动学、XML/YAML/SDF/launch 检查。运行验收时应分别测试：
全向的 X/Y/yaw 三自由度（当前导航只支持 omni）；以及
`CustomMsg` 字段、Point-LIO 静止稳定性和真值位姿。注意真值里程计记录底盘模型原点，
Point-LIO 记录融合 IMU 原点，比较平面真值前必须施加 map 变换和上述 0.2 m 原点修正；
运行验收仍需分别检查静止 Z/roll/pitch 峰峰值、中央区域漂移和 ROGMap 地面残影。

当前 RMUC 2026 临时网格因可见坑洞尚未通过上述运行验收。外部修复网格替换完成前，RMUC 2026 只用于检查资源加载、坐标链和转换流程，不用于评价最终坡道通过性或定位精度。
