# Odin1 Lite / O1LITE 传感器仿真

面向 ROS 2 Humble + Gazebo Fortress 的可挂载传感器资源。依据用户提供的
《Odin1Lite 用户手册 V0.1.4》（2026-08-13）制作简化外壳、dToF 点云、RGB 与 IMU；
模型为本仓库创建的近似模型，没有复制厂商 CAD、固件或 PDF。

外观参考手册前视图：石墨灰倒角外壳、双 dToF 窗口、下方 RGB 镜头、金属压圈、
四角螺钉、散热片、状态灯和后部接头。配色与小细节是装饰性近似，状态灯不表示
真实设备状态；窗口位置也不作为标定依据。装饰只用于 visual，物理层仍使用原
58×69×69 mm 碰撞盒和均匀盒体惯量，传感器外参独立保存在下方 frame 中。
`meshes/housing.stl` 与 `meshes/cooling_ribs.stl` 为米制原创低面数网格，共 396 个三角形；
散热片合并为一个 visual，其余镜头和螺钉使用 Gazebo 原生几何，无贴图或额外依赖。

## 启动独立演示

在 ROS 工作空间根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select sentry_simulation
source install/setup.bash
ros2 launch sentry_simulation odin1_lite.launch.py
# 不启动 Gazebo GUI（相机与 GPU LiDAR 仍需要可用的渲染环境）：
# ros2 launch sentry_simulation odin1_lite.launch.py headless:=true
```

演示把模组固定在世界坐标 `(0, 0, 1)`，前方放置一个前表面位于 `x=3 m`
的红色靶板。启动传感器、ROS bridge、仿真时钟和静态 TF，不启动定位或导航。
在 RViz 中设置 Fixed Frame 为 `world`，添加 PointCloud2 和 Image 显示即可查看。
这是独立世界入口；不要与另一个向同一 ROS domain 发布 `/clock` 的仿真实例同时启动。

## 文件与接口

| 文件（相对 `sentry_simulation`） | 用途 |
|---|---|
| `resource/models/odin1_lite/model.config` | Gazebo 资源元信息 |
| `resource/models/odin1_lite/model.sdf` | 物理外壳、三个传感器、内部坐标系与外参 |
| `resource/worlds/odin1_lite_demo.sdf` | 独立演示、光照、地面和靶板 |
| `config/odin1_lite_bridge.yaml` | Gazebo 到 ROS 的单向桥接 |
| `launch/odin1_lite.launch.py` | 演示入口；直接读取 SDF 外参发布静态 TF |

| ROS 话题 | 消息 | frame_id | 配置频率（仿真时间） |
|---|---|---|---|
| `/sim/odin1_lite/points` | `sensor_msgs/msg/PointCloud2` | `odin1_lite_lidar` | 10 Hz |
| `/sim/odin1_lite/imu` | `sensor_msgs/msg/Imu` | `odin1_lite_body` | 200 Hz |
| `/sim/odin1_lite/rgb/image` | `sensor_msgs/msg/Image` | `odin1_lite_camera_optical` | 30 Hz |
| `/sim/odin1_lite/rgb/camera_info` | `sensor_msgs/msg/CameraInfo` | `odin1_lite_camera_optical` | 随相机 |
| `/clock` | `rosgraph_msgs/msg/Clock` | — | 随物理步进 |

订阅端使用 `use_sim_time:=true`；点云在 Gazebo 侧的话题是
`/sim/odin1_lite/lidar/points`。资源的 topic/frame 名称固定，当前支持单模组；多模组
需要同时修改各实例的传感器 topic/frame、桥接和 TF，单纯改 include 名字不能隔离它们。

一帧点云约 1.38 MB，RGB 约 4.18 MB。入口默认复用包内 `fastdds_shm.xml` 的
16 MiB 共享内存设置；已有 `FASTRTPS_DEFAULT_PROFILES_FILE` 时保留用户配置。
设置 `ROS_LOCALHOST_ONLY=1` 时不会自动加载这个含自定义 UDP 的配置，以保留
localhost 限制；此时大消息传输可能丢帧，需要用户提供适合本机的 DDS 配置。

## 手册参数与近似范围

| 项目 | 采用值 | 依据与边界 |
|---|---|---|
| 外壳与质量 | 宽 69、高 69、深 58 mm；274 g | 手册 §2、§12；简化盒体，不含接头，原点/质心和均匀盒体惯量是近似 |
| dToF 视场 | 水平 120°，垂直 90° | §12；用均匀角度 GPU 射线模拟，非真实面阵投影 |
| dToF 分辨率 | 240×180，10 Hz | §12；§5.4 原始点云协议另写 256×192，两处不一致，本资源选择规格表 |
| dToF 距离 | 0.15～25 m | §12 的 25 m 有反射率与照度条件，50 m 为另一条件；仿真仅采用固定截断 |
| dToF 噪声 | 标准差 0.03 m 的高斯距离噪声 | 仅近似 §12 的 5 m 精度条件，不复现距离/材质/光照依赖 |
| RGB | 1280×1088，水平 121°、垂直 106° | §12；本资源使用去畸变针孔近似，不生成原始 FishPoly 鱼眼图像 |
| RGB 帧率 | 30 Hz | 演示选择，手册未明确给出额定 RGB 帧率 |
| IMU | 200 Hz，理想六轴输出 | 手册未给采样率及噪声参数；此频率是仿真选择，静止加速度约 +9.81 m/s² Z |

RGB 的针孔内参由上述视场和图像尺寸计算：
`fx=362.094578040, fy=409.933403256, cx=640, cy=544`。
这不是实机的 FishPoly 标定，不能直接把实机畸变系数用于该图像。
修改相机分辨率或视场时需要同步修改 SDF 中的内参。

Gazebo 输出标准 `PointCloud2`，没有复现真机 §5.4 的 18 字节点布局
（特别是 `confidence`、逐点 `offset_time`），也没有 dToF 多径、运动扫描时序、
反射率或强光干扰模型。该资源不发布 `/odin1/cloud_slam`、MindSLAM 位姿或地图，
不能仅重映射 topic 就替代当前双 MID360 的 `CustomMsg` / Point-LIO 输入链。

## 坐标系与标定

`odin1_lite_body` 是模组 Body/IMU 原点，X 前、Y 左、Z 上。
`odin1_lite_lidar` 的外参来自手册 §5.2（PDF 第 12 页）的 `body_T_lidar`。
相机外参由第 13 页示例设备 `O1L-P040100047` 的 `camera_T_lidar` 逆变换与
`body_T_lidar` 合成；打印旋转矩阵经离线正交化后转为 RPY。
这些是手册示例，不能当作用户设备的实测外参。

Gazebo 相机本体帧 `odin1_lite_camera` 使用 X 前、Y 左、Z 上；
ROS 光学帧 `odin1_lite_camera_optical` 使用 X 右、Y 下、Z 前。
SDF 内部 frame 同时用于传感器 pose 和演示 TF，避免维护两份外参。
有本机 `camera_calib_<SN>.yaml` 后，可换算并更新 SDF 中的相关 pose。

## 挂载到现有机器人

在机器人 SDF 的 `<model>` 内 include 模组，并固定到实际车体 link，例如：

```xml
<include>
  <uri>model://odin1_lite</uri>
  <name>odin1_lite</name>
  <pose relative_to="base_link">0.2 0 0.4 0 0 0</pose>
</include>
<joint name="odin1_lite_mount" type="fixed">
  <parent>base_link</parent>
  <child>odin1_lite::odin1_lite_body</child>
</joint>
```

安装位置只是示例，需按车辆测量修改。宿主 world 需要 Sensors（Ogre2）和 Imu
系统插件，写法可直接参考演示 world。使用本资源 bridge 中四个传感器映射；
宿主已有 `/clock` 桥接时保留原桥接即可。由机器人 TF 链维护车体到模组 Body
及内部传感器的变换，不要同时运行演示 launch 的 `world -> odin1_lite_body` 静态 TF。
本次提供资源和挂载方法，现有整车及默认导航启动入口没有接入这个模组。

`model.sdf` 本身是动态模型；只有演示 world 的 include 设置了 `static=true`。
新增资源后重新构建并 source overlay，Gazebo 才能通过 `model://odin1_lite` 找到模型。
