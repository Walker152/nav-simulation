# Sentry simulation

这个模块将 Gazebo Fortress 仿真资源收敛到当前仓库，并直接接入现有导航主链路：

```text
Gazebo 左右双 MID360（每颗由前/后 180° GPU LiDAR 拼接）+ 水平 IMU
  -> ros_gz_bridge
  -> 双雷达公共帧融合 + MID360 pattern adapter
     (/livox/lidar: livox_ros_driver2/msg/CustomMsg)
  -> Point-LIO (/aft_mapped_to_init, /cloud_registered_full)
  -> model PCD GICP (pcd_map -> camera_init, 2025 场地)
  -> ROGMap + MINCO planner
  -> Minco MPC controller (/cmd_vel_mpc, 世界坐标系)
  -> sentry_sim_cmd_adapter
  -> Gazebo omni/diff chassis
```

仿真不会启动 `communication`，也不会使用真车裁判系统或下位机链路。Gazebo 的
`/sim/ground_truth/odom` 只用于验真，不会替代 Point-LIO 里程计作为导航输入。

## 已包含的资源

- RMUC / RMUL 2024、2025 四套场地 SDF、网格与二维地图。
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

在仓库根目录构建示例：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select sentry_simulation
```

启动脚本会检查 `sentry_simulation`、ROS–Gazebo bridge、`point_lio`、
`icp_relocalization`、`navi2`、
`minco_planner`、`minco_controller` 和 `rog_map`。底盘只使用 Fortress 原生驱动；
历史 `MecanumDrive2` 源文件保留作追溯，但不会构建、安装或在启动时检查。

## 启动

```bash
# 全向底盘 + RMUC 2025（默认）
./simlation.bash omni rmuc_2025

# 差速底盘 + RMUL 2025
./simlation.bash diff rmul_2025

# 无 Gazebo GUI，也不启动 RViz
./simlation.bash omni rmuc_2024 --headless --no-rviz
```

可选 world：`rmuc_2024`、`rmul_2024`、`rmuc_2025`、`rmul_2025`。
2025 场地默认启用模型 PCD GICP；2024 场地因现有 PCD 与旧场地几何不一致，保持
实车初值静态定位，不执行可能误收敛的 ICP。所有随包 PGM YAML 原点均为 `[0, 0, 0]`。
RMUL 2025 使用与模型 PCD 同源的 2026 RMUL PGM，并把居中的场地网格平移到零原点地图。

启动后可依次检查：

```bash
ros2 topic hz /livox/lidar
ros2 topic info /livox/lidar
ros2 topic hz /sim/imu
ros2 topic hz /aft_mapped_to_init
ros2 topic hz /cloud_registered_full
ros2 topic hz /cmd_vel_mpc
ros2 topic echo /sim/ground_truth/odom --once
```

在 RViz 中用 Nav2 的 `2D Goal Pose` 发目标，即可测试完整规划和控制闭环。

## 坐标、时间与速度语义

- 全部节点启用 `/clock` 和 `use_sim_time`。
- 左右 MID360 相对公共帧的位姿分别为
  `(-0.0496, +0.136, 0, -0.5835988, 0, 0)` 和
  `(-0.0496, -0.136, 0, +0.5835988, 0, 0)`；公共 LiDAR/IMU 帧位于车体中心
  `z=0.25 m`，保持水平。适配器先把两颗雷达变换到该公共帧，再发布一条 CustomMsg。
- 2025 场地通过 `map -> pcd_map -> camera_init` 接入一次性 GICP；超时会回退到同一
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

## 当前验证边界

模块提供静态契约、Python 运动学、XML/YAML/SDF/launch 检查。运行验收时应分别测试：
全向的 X/Y/yaw 三自由度；差速对车体系 `linear.y` 的拒绝、前进和原地旋转；以及
`CustomMsg` 字段、Point-LIO 静止稳定性和真值位姿。注意真值里程计记录底盘模型原点，
Point-LIO 记录双雷达融合后的车体中心公共帧，因此可直接和底盘真值比较平面位置；
运行验收仍需分别检查静止 Z/roll/pitch 峰峰值、中央区域漂移和 ROGMap 地面残影。
