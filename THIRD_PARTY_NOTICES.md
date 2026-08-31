# Third-party simulation resources

The following files were migrated without changing their upstream license:

- `sentry_simulation/resource/models/rmuc_2024`, `rmul_2024`, `rmuc_2025`,
  and `rmul_2025`, plus the matching world files: from
  `RoboMaster/Simulation/rmu_gazebo_simulator`, revision `41e6fe4`, Apache-2.0.
- `sentry_simulation/maps`: from `pb2025_sentry_nav`, revision
  `b5c6a5200a10bf9d7c6f1aa72358afcdfab84b05`, Apache-2.0.
- `sentry_simulation/resource/maps/pcd/rmuc_model.pcd`: copied from this
  workspace's `icp_relocalization/pcd/cloudlab_processed.pcd`; it is the
  compact model cloud, not the red/blue real-vehicle clouds.
- `sentry_simulation/resource/maps/pcd/rmul_model.pcd` and the aligned
  `sentry_simulation/maps/rmul_2025.pgm`: copied from this workspace's
  `navi2_bringup/maps/pcd/2026rmul.pcd` and `maps/2026/rmul2026.pgm`.
- The point-cloud field conversion behavior is adapted from
  `pb2025_sentry_nav/ign_sim_pointcloud_tool`, and the world-to-body velocity
  conversion follows `pb2025_sentry_nav/fake_vel_transform`, at the same
  revision above, Apache-2.0.
- `sentry_simulation/plugins/mecanum_drive2`: from RoboMaster-OSS
  `rmoss_gz_plugins`, Apache-2.0. Its source files retain their copyright and
  license headers.
- `sentry_simulation/resource/models/mid360/meshes/mid360.dae`: copied from
  the local `pb2025_robot_description` repository, revision `9afbce3`, whose
  package declares MIT.
- `sentry_simulation/resource/models/mid360/scan_mode/mid360-real-centr.csv`:
  from the ROS 2 branch of `Tfly6/Mid360_px4_sim_plugin`; that package declares
  Apache-2.0. Source URL:
  `https://github.com/Tfly6/Mid360_px4_sim_plugin/tree/ros2`.
  The imported CSV SHA-256 is
  `aa486c66092af45eab59fccb1119421470ba01981434b75d04367dd374576553`.
- `sentry_simulation/resource/models/pb2025_visuals/meshes`: unmodified
  `chassis_base.dae`, `gimbal_yaw.dae`, and `gimbal_pitch.dae` assets from
  `SMBU-PolarBear-Robotics-Team/rmoss_gz_resources`, branch `humble`, as
  referenced by `pb2025_robot_description/dependencies.repos`. The upstream
  repository declares Apache-2.0; its license is retained beside the assets.

The adapters, launch files, simplified sentry models, and configuration added
in this module are part of this repository and use Apache-2.0.
