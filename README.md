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
  -> Minco MPC controller (/cmd_vel_mpc, 世界坐标系)
  -> sentry_sim_cmd_adapter
  -> Gazebo omni/diff chassis
```

仿真不会启动 `communication`，也不会使用真车裁判系统或下位机链路。Gazebo 的
`/sim/ground_truth/odom` 只用于验真，不会替代 Point-LIO 里程计作为导航输入。

## 功能和边界

| 功能 | 实现 | 说明 |
|---|---|---|
| 全向底盘 | `sentry_omni` + Fortress `MecanumDrive` | 接收转换后的车体系速度，可测试 X/Y/yaw |
| 差速底盘 | `sentry_diff` + Fortress `DiffDrive` | 将世界系 MPC 命令转换为前进/转向，允许倒车 |
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
  Fortress 原生 `MecanumDrive`，PB2025 chassis/gimbal 网格仅作视觉模型。
- `sentry_diff`：半径 `0.3 m` 的圆形两轮差速底盘，仅左右两轮驱动，前后球形
  万向支承，使用 Fortress 原生 `DiffDrive`。
- 双 MID360：左右镜像安装在 `z=0.25 m`，每颗水平 360°、垂直
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
`minco_planner`、`minco_controller` 和 `rog_map`。底盘只使用 Fortress 原生驱动；
历史 `MecanumDrive2` 源文件保留作追溯，但不会构建、安装或在启动时检查。

## 快速启动

所有命令都在父仓库根目录执行。先运行预检，不启动 Gazebo：

```bash
cd /home/alioth/2025-sentry-navi
./simlation.bash omni rmuc_2025 --check
```

脚本位置参数和选项：

| 参数 | 可选值 | 默认值 | 作用 |
|---|---|---|---|
| 第 1 个位置参数 | `omni`、`diff` | `omni` | 选择全向或差速底盘 |
| 第 2 个位置参数 | `rmuc_2024`、`rmul_2024`、`rmuc_2025`、`rmuc_2026`、`rmul_2025` | `rmuc_2025` | 选择场地 |
| `--headless` | 开关 | 关闭 | 不启动 Gazebo GUI，只运行 server |
| `--no-rviz` | 开关 | 关闭 | 不启动 RViz |
| `--check` | 开关 | 关闭 | 只检查 ROS overlay 和依赖包 |

常用启动方式：

```bash
# 全向底盘 + RMUC 2025（默认）
./simlation.bash omni rmuc_2025

# 全向底盘 + RMUC 2026 点云重建场地
./simlation.bash omni rmuc_2026

# 差速底盘 + RMUL 2025
./simlation.bash diff rmul_2025

# 无 Gazebo GUI，也不启动 RViz
./simlation.bash omni rmuc_2024 --headless --no-rviz
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

直接 `ros2 launch` 使用 install tree；新增或替换资源后应重新执行 symlink-install 构建。根目录 `simlation.bash` 会额外指定源码侧 simulation share 和模型搜索路径，适合当前仓库日常启动。

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
| `/cmd_vel_mpc` | `geometry_msgs/msg/Twist` | MPC 输出的世界系速度 |
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
`/aft_mapped_to_init`；适配器必须先收到有效里程计，才能把世界系速度旋转到车体系。

## 坐标、时间与速度语义

- 全部节点启用 `/clock` 和 `use_sim_time`。
- 左右 MID360 相对公共帧的位姿分别为
  `(-0.0496, +0.136, 0, -0.5835988, 0, 0)` 和
  `(-0.0496, -0.136, 0, +0.5835988, 0, 0)`；公共 LiDAR/IMU 帧位于车体中心
  `z=0.25 m`，保持水平。适配器先把两颗雷达变换到该公共帧，再发布一条 CustomMsg。
- 2025/2026 场地通过 `map -> pcd_map -> camera_init` 接入一次性 GICP；超时会回退到同一
  实车初始位姿。2024 场地直接发布 `map -> camera_init`。两种路径的定位原点 Z 都为零。
- planner/controller 的雷达杆臂补偿设为零，因为 Point-LIO 输出已经位于车体中心公共帧。
- Nav2 使用同时包络全向轮和差速轮的凸多边形 footprint；MINCO corridor 半径为
  `0.42 m`、优化安全距离为 `0.45 m`，避免原先 `0.20/0.25 m` 低估车体后卡场地边角。
- Point-LIO 继续发布当前仓库既有的 `camera_init -> aft_mapped` 与里程计话题。
- MPC `/cmd_vel_mpc` 是世界坐标系速度；适配节点通过 Point-LIO yaw 转为车体系。
- 差速模式不会把 `linear.y` 直接发送给底盘，而是生成转向角速度；目标在车后方时允许倒车。
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

新增其他场地时，复制现有同类型目录并在 `config/worlds.yaml` 注册；随后同步更新根目录 `simlation.bash` 的 world 白名单。场地名称、目录名、model URI 和 catalog key 应保持一致。

## 常见问题

### `Unsupported world` 或 `unknown world`

前者来自根脚本白名单，后者来自 `config/worlds.yaml`。确认两个位置都注册了同一个场地名，并检查 world/map 文件路径。

### Gazebo 找不到 `model://...`

优先使用根目录 `simlation.bash`，它会设置 `IGN_GAZEBO_RESOURCE_PATH`、`GZ_SIM_RESOURCE_PATH`、`SDF_PATH` 和 `IGN_FILE_PATH`。直接 launch 时需要重新构建并 source 最新 install tree。

### 有导航目标，但小车不动

依次检查 `/aft_mapped_to_init`、`/cmd_vel_mpc`、`/sim/cmd_vel`。没有里程计时速度适配器不会转发命令；只有 `/cmd_vel_mpc` 没有 `/sim/cmd_vel` 时，重点检查适配器和里程计；两者都有但 Gazebo 不动时，再检查底盘 plugin 和实体名称。

### 里程计 Z 发散或 ROGMap 把地面投成障碍

先比较 `/aft_mapped_to_init` 与 `/sim/ground_truth/odom`，再检查 IMU、双雷达时间戳、模型碰撞面和平地高度。碰撞网格的坑洞、假坡和薄板塌缩会直接造成车体姿态抖动，不能只通过 Point-LIO 参数掩盖。

### GICP 没有接受结果

确认目标 PCD 与 world 是同一场地、单位和坐标方向，检查 `worlds.yaml` 中的 `pcd_to_map`、`min_inlier_ratio` 和 `min_overlap_ratio`。调低比例阈值之前先查看实际重叠和几何一致性；超时回退只保证初值存在，不代表模型点云已经正确对齐。

## 当前验证边界

模块提供静态契约、Python 运动学、XML/YAML/SDF/launch 检查。运行验收时应分别测试：
全向的 X/Y/yaw 三自由度；差速对车体系 `linear.y` 的拒绝、前进和原地旋转；以及
`CustomMsg` 字段、Point-LIO 静止稳定性和真值位姿。注意真值里程计记录底盘模型原点，
Point-LIO 记录双雷达融合后的车体中心公共帧，因此可直接和底盘真值比较平面位置；
运行验收仍需分别检查静止 Z/roll/pitch 峰峰值、中央区域漂移和 ROGMap 地面残影。

当前 RMUC 2026 临时网格因可见坑洞尚未通过上述运行验收。外部修复网格替换完成前，RMUC 2026 只用于检查资源加载、坐标链和转换流程，不用于评价最终坡道通过性或定位精度。
