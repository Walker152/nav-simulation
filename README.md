# Sentry simulation

这个模块将 Gazebo Fortress 仿真资源收敛到当前仓库，并直接接入现有导航主链路：

```text
Gazebo GPU LiDAR + IMU
  -> ros_gz_bridge
  -> pointcloud_adapter (/velodyne_points: x/y/z/intensity/ring/time)
  -> Point-LIO (/aft_mapped_to_init, /cloud_registered_full)
  -> ROGMap + MINCO planner
  -> Minco MPC controller (/cmd_vel_mpc, 世界坐标系)
  -> sentry_sim_cmd_adapter
  -> Gazebo omni/diff chassis
```

仿真不会启动 `communication`，也不会使用真车裁判系统或下位机链路。Gazebo 的
`/sim/ground_truth/odom` 只用于验真，不会替代 Point-LIO 里程计作为导航输入。

## 已包含的资源

- RMUC / RMUL 2024、2025 四套场地 SDF、网格与二维地图。
- `sentry_omni`：四轮全向底盘，使用迁移后的 `MecanumDrive2` 插件。
- `sentry_diff`：四轮差速底盘，使用 Gazebo Fortress `DiffDrive` 插件。
- 32 线、20 Hz GPU LiDAR 和 200 Hz IMU。
- Point-LIO 标准 `PointCloud2` 字段适配器及仿真参数。
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
colcon build --symlink-install --packages-up-to sentry_simulation
```

本次 Agent 修改没有执行上述安装或构建命令。

启动脚本会检查 `sentry_simulation`、ROS–Gazebo bridge、`point_lio`、`navi2`、
`minco_planner`、`minco_controller` 和 `rog_map`，全向模式还会检查安装后的
`libMecanumDrive2.so`。缺少任一项时会在启动 Gazebo 前给出明确错误。

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
默认出生点已按各自 PGM 地图校正到至少 0.25 m 半径的 free 区域；上游 RMUC 2025
和 RMUL 2025 的红方出生点位于其随包地图边界之外，因此分别使用地图内出生点和上游
蓝方出生点。

启动后可依次检查：

```bash
ros2 topic hz /sim/lidar/points
ros2 topic hz /sim/imu
ros2 topic hz /aft_mapped_to_init
ros2 topic hz /cloud_registered_full
ros2 topic hz /cmd_vel_mpc
ros2 topic echo /sim/ground_truth/odom --once
```

在 RViz 中用 Nav2 的 `2D Goal Pose` 发目标，即可测试完整规划和控制闭环。

## 坐标、时间与速度语义

- 全部节点启用 `/clock` 和 `use_sim_time`。
- 启动文件根据场地出生点建立 `map -> camera_init` 静态变换。
- Point-LIO 继续发布当前仓库既有的 `camera_init -> aft_mapped` 与里程计话题。
- MPC `/cmd_vel_mpc` 是世界坐标系速度；适配节点通过 Point-LIO yaw 转为车体系。
- 差速模式不会把 `linear.y` 直接发送给底盘，而是生成转向角速度；目标在车后方时允许倒车。
- MPC 命令超过 0.25 s 未刷新时，适配节点会向 Gazebo 发送一次零速度，避免底盘保持旧指令。
- 点云 `time` 字段单位是秒，所以 `timestamp_unit` 固定为 `0`，不要照搬旧仿真配置中的微秒设置。

## STEP / CAD 资源是否需要

跑通定位、规划和控制闭环不需要 STEP：当前简化模型已经包含合理的外形碰撞体、质量、
轮距、轮径和传感器位姿，场地也已有可用网格。

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

模块已提供静态契约、Python 运动学、XML/YAML 和 launch 语法检查。只有在本机依赖安装、
允许执行 `colcon build` 并实际启动 Gazebo 后，才能确认 Ignition 插件 ABI、渲染驱动、
传感器频率和完整闭环的运行表现。
