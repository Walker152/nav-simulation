# 四舵轮、四 O1LITE 独立仿真

用于当前 Naturewill 的 ROS 2 Humble / Gazebo Fortress 6（本机 6.18.0），
依据工作空间 `gazebo-model-handoff.md` 建模。独立运行，不启动导航、Point-LIO、
仲裁器或任何 EtherCAT/实机节点。Jazzy 工程的纯 `kinco_swerve_core` 已带来源复制；
执行器使用本包 `SwerveDrive` Fortress 物理插件，并非原 Jazzy ros2_control 控制器。

## 构建、启动

```bash
cd /home/alioth/nature_will
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select sentry_simulation
source install/setup.bash
ros2 launch sentry_simulation swerve_odin.launch.py rviz:=true
```

无头：`headless:=true`；底盘单独检查：再加 `sensors:=false`（外壳和质量仍保留）。
Gazebo 的 RGB/dToF 仍需要可用的图形渲染环境；`headless` 只关闭 GUI。
退出用 Ctrl+C，launch 同时停止 Gazebo、桥接和 TF 节点并清理临时生成资源。
与其他仿真同时运行时，使用独立 `ROS_DOMAIN_ID` 和 `IGN_PARTITION`。

配置由 `config/swerve_odin.yaml` 唯一生成 SDF、URDF 和桥接表。自定义完整 YAML：
`config:=/absolute/path/to/swerve.yaml`；不隐式叠加另一工作空间参数。
生成文件位于本次 launch 的 `/tmp/swerve_odin_*`，退出时删除；安装后的启动不依赖源码目录。
生成器 `sentry_simulation/swerve_platform.py` 也可独立导出模型供检查。

## 安装与模型约定

X 前、Y 左、Z 上，`base_link` XY 为四转向轴中心。外廓 0.804 × 0.604 m；
四模块 FL/FR/RL/RR 中心为 (±0.340, ±0.240) m。轮半径 0.065 m，接触宽 0.024 m；
厂家 0.030 m 是总轮宽。模型无悬挂、无转向轴/轮心偏置。

| O1LITE | body/IMU 相对 base_link 的 XYZ/m | yaw/rad | ROS 话题前缀 |
|---|---|---|---|
| 前 | (0.402, 0, 0.060) | 0 | `/sim/odin_front` |
| 后 | (-0.402, 0, 0.060) | π | `/sim/odin_rear` |
| 左 | (0, 0.302, 0.060) | π/2 | `/sim/odin_left` |
| 右 | (0, -0.302, 0.060) | -π/2 | `/sim/odin_right` |

四路均发布 `/points`、`/imu`、`/rgb/image`、`/rgb/camera_info`，对应 frame 为
`odin_<方向>_lidar`、`odin_<方向>_body`、`odin_<方向>_camera_optical`。
安装点随 YAML 长宽自动变化，Z 是可调安装假设。
外壳装饰、外参和传感器规格来自同一个 [O1LITE 资源](resource/models/odin1_lite/README.md)。
240×180 / 10 Hz dToF、1280×1088 / 30 Hz RGB、200 Hz IMU；频率按仿真时间，
四套图像/点云的 GPU 负载可能使仿真慢于实时。外参是手册示例，RGB 是针孔近似，
没有硬件原始包、置信度、FishPoly 或设备内 MindSLAM。这里的点云是 dToF 模拟。

100 kg 是包含所有 link 的假设总质量：每模块 3.2 kg 转向件 + 1.5 kg 轮，
四 Odin 各 0.274 kg，余下车体 80.104 kg。惯量由选定箱体/圆柱质量计算。
车体箱高 0.110 m、中心 z=-0.040 m，轮心 z=-0.120 m；落地 base_link 高约0.185 m，
箱底约0.090 m。它们是占位，未匹配实物约0.080 m离地间隙，也不等于30 mm主底板厚度。
摩擦、接触刚度/阻尼、关节阻尼、转向件质量与几何均未标定，见 YAML。
ODE 接触参数能否参与求解取决于 Gazebo 物理后端，不视为经过辨识的材料参数。

## 命令、反馈与限幅

唯一输入 `/kinco_swerve/cmd_vel/selected`，`geometry_msgs/msg/TwistStamped`。
独立测试时只启动一个发布源。使用仿真时间，frame 为空或 `base_link`，
所有分量有限；正且非未来的时间戳，有效期默认0.25 s。正常新鲜零命令平滑停车；
零时间戳、过期、未来、非法数据立即撤销轮速目标，锁存当时实测舵角并清参考状态。
有限力矩制动仍需物理时间。暂停和世界 reset 丢弃缓存，恢复必须收到新命令。
Fortress reset 后轮式积分从零开始，但物理轮速可能保留并经历制动；
本次reset后0.3s轮式积分新增约0.038m，不把reset当成瞬时物理停车。

示例前进0.3 m/s，Ctrl+C后超时停车：

```bash
ros2 topic pub -r 50 /kinco_swerve/cmd_vel/selected geometry_msgs/msg/TwistStamped \
  '{header: {stamp: now, frame_id: base_link}, twist: {linear: {x: 0.3}}}' \
  --ros-args -p use_sim_time:=true
```

八轴链为 `base_link → *_steer_link → *_wheel_link`，关节名 `*_steer_joint`、
`*_wheel_joint`，前缀 `front_left/front_right/rear_left/rear_right`。
舵轴局部Z、轮轴转向后局部Y；正轮速沿模块+X。转向目标±155°，机械端点假设±168°。
500 Hz物理/控制，100 Hz发布；有限力矩伺服，不直接重置关节位置或施加底盘速度。

| 输出 | 含义 |
|---|---|
| `/swerve/joint_states` | 八轴物理位置/速度反馈；effort不作为测得力矩使用 |
| `/swerve/joint_targets` | 舵角/舵速参考与轮速目标；轮位置字段不使用 |
| `/swerve/reference_twist` | 车体速度/加速度整形后的参考，尚未乘舵向对齐或轮速饱和比例 |
| `/swerve/wheel_odometry` | 根据实测轮速和舵角估计并积分，打滑会漂移，不发布TF |
| `/swerve/ground_truth/odometry` | Gazebo物理位姿真值，odom原点与world重合 |
| `/tf`、`/tf_static` | 真值插件唯一发布odom→base_link，RSP发布其下关节/传感器树 |

平移合速度2 m/s、偏航2 rad/s、轮缘3 m/s；平移合加速度1 m/s²、角加速度2 rad/s²；
舵向参考速度2π rad/s、参考加速度4π rad/s²。组合运动使用四轮共同缩放；
舵向未对齐时共同降低轮速，防止转向过程中直接推进。实际速度、加速度受质量、
摩擦、力矩和瞬态误差影响，应读真值输出，不能把参考限制当实测物理上界。

## 复验

```bash
colcon test --packages-select sentry_simulation
colcon test-result --test-result-base build/sentry_simulation
# 另一个终端已启动独立仿真；两个终端的 ROS_DOMAIN_ID 相同
ros2 run sentry_simulation validate_swerve --mode sensors --output /tmp/swerve-sensors.json
# 底盘运动检查推荐 sensors:=false；不要连接真实机器人
ros2 run sentry_simulation validate_swerve --mode motion --output /tmp/swerve-motion.json
```

运动脚本按默认YAML的验收数值检查；修改限值后需相应调整测试目标。
脚本订阅实际数据检查四路传感器/TF或前后、左右、斜移、旋转、组合、普通与失效停车，
输出 JSON 证据；动力学尚未标定，这些结果仅用于该仿真模型。
本次实测结果、截图及已知边界见工作空间 `src/docs/ai_refactor_records/20260921_swerve_odin_platform.md`。

本次默认配置的实际验证：26个新增单元/模型检查通过；实际前后、横移、斜移、
旋转、组合及失效停车通过。直线2m/s参考实测约1.98m/s，yaw2rad/s参考
稳态约1.9rad/s。普通参考加速度实测1m/s²、2rad/s²，舵向参考加速度12.5664rad/s²；
整个测试含失效制动的实际加速度峰值约4.84m/s²、8.26rad/s²。
另设轮缘上限0.3m/s时，四轮共同缩放比例0.394617、最大轮速4.61538rad/s。
全包旧simulation_contract另有12个基线失败（已删除导航参数文件/旧预期），
未混入本次独立平台修改；本次通过结论只针对上述范围。
