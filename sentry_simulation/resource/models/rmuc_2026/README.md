# RMUC 2026 CAD 场地资产

本模型由用户提供的 `Downloads/RMUC2026.stl` 导入（2026-09-20），替代旧的点云重建视觉/碰撞网格。Gazebo 中的资源名继续使用 `rmuc_2026`，脚本和 launch 默认选择它。

## 来源与坐标

- 原始文件：`/home/alioth/Downloads/RMUC2026.stl`，377,347,784 bytes，7,546,954 个三角面；原文件未改动。
- 原始 SHA-256：`735a980464fe22441bed887e47267a3c7b8b0faa0b43ba78bd3685ac678be754`。
- 源文件未附作者与许可说明，不推定其作者或开源许可证。
- 资产：`meshes/rmuc_2026.stl`，25,000,084 bytes，500,000 个三角面；视觉和碰撞引用同一个文件，避免几何不一致与重复资源。
- 资产 SHA-256：`732759272e28f327196da78c0068600f9e8364185f8d68a3b9c719cc859988d5`。
- 转换：`map_xyz = source_xyz * 0.001 + [14.58, 5.82, 1.74134363]`，Z-up，无旋转。STL 已烘焙米制坐标，SDF 的 scale 为 `1 1 1`，world pose 为零。
- 资产边界约为 `[-0.296, -0.557, -0.200]` 到 `[29.456, 15.446, 3.601]` m。

对齐依据为包内 `resource/maps/pcd/rmuc_2026.pcd`：CAD 表面均匀采样 30 万点，在 6 cm 下采样后，以平移初值 `[14.55, 5.875, 1.74134875]` 执行 point-to-plane ICP，依次使用 0.5/0.2/0.08 m 对应阈值（每级最多 60 次迭代）。最终 8 cm 内对应比例约 86.67%，RMSE 4.23 cm，拟合平移 `[14.57878, 5.81822, 1.75157]`；总旋转低于 0.01°。保持 CAD 坐标轴，XY 取厘米精度；Z 使 CAD 底板底面对应现有点云的 -0.20 m。场地具有近似中心对称性，采用与原始 CAD 同向的解。

新 CAD 与既有点云并非逐点一致（例如中央机构细节），上述数值为整体几何对齐证据，不代表导航定位精度。二维地图和 GICP 目标 PCD 保持现有版本，`pcd_to_map` 仍为零。

## 离线生成复现

仅资产制作需要 Open3D 0.19.0（本次使用 NumPy 1.26.4）；仿真启动不依赖 Open3D，不读取 Downloads。以下脚本从原始文件生成随包资产，在仓库 simulation 目录执行：

```python
from pathlib import Path
import open3d as o3d

source = Path('/home/alioth/Downloads/RMUC2026.stl')
target = Path('sentry_simulation/resource/models/rmuc_2026/meshes/rmuc_2026.stl')
mesh = o3d.io.read_triangle_mesh(str(source))
mesh.scale(0.001, center=(0, 0, 0))
mesh.remove_duplicated_vertices()
mesh.remove_duplicated_triangles()
mesh.remove_degenerate_triangles()
mesh.remove_unreferenced_vertices()
mesh = mesh.simplify_quadric_decimation(500000, maximum_error=0.000001)
# 与首次导入一致：简化结果先以 binary STL 保存/读取，固定浮点精度。
intermediate = Path('/tmp/rmuc_2026_cad_metres.stl')
mesh.compute_vertex_normals()
assert o3d.io.write_triangle_mesh(str(intermediate), mesh)
mesh = o3d.io.read_triangle_mesh(str(intermediate))
mesh.translate([14.58, 5.82, 1.74134363])
mesh.compute_vertex_normals()
assert o3d.io.write_triangle_mesh(str(target), mesh)
```

`maximum_error` 是简化算法的误差指标，并非逐点距离上限。原始 CAD 的 30 万个均匀表面样本到简化网格的距离 P95 为 0.0084 mm、P99 为 0.506 mm，最大为 28.9 mm；最大误差不能视作坡面误差或全模型保证。没有为减少面数删除地面或用凸包替换整个场地。

## 出生点与验证边界

出生 XY/yaw 保留 `[4.234, 7.3, 0.04]`。此处真实地面约 z=0.096 m，四轮附近约 0.090–0.098 m，故 `worlds.yaml` 的出生高度设为 0.11 m，给轮底留下间隙。不能沿用原临时网格的 z=0，否则车轮会嵌入新 CAD 地形。

实测90秒无界面默认启动进入导航active并确认GICP；20秒静止Gazebo真值XY峰峰值低于0.6mm，未坠落。初次导入时发现Point-LIO静止漂移，后续定位为落地冲击污染重力初始化，并通过生成后等待1秒仿真时间再启动定位修复；完整动态定位精度仍需验收；停止时容器段错误在旧2025场地亦可复现，未在本次修改导航/退出逻辑。

资源解析、构建、默认入口和实际启动的检查见主仓 `docs/ai_refactor_records/20260920_rmuc2026_stl_scene.md`。全赛道坡道/台阶通过性、长时定位和导航性能需要独立验收。
