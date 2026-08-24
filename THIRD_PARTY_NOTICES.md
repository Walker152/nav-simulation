# Third-party simulation resources

The following files were migrated without changing their upstream license:

- `sentry_simulation/resource/models/rmuc_2024`, `rmul_2024`, `rmuc_2025`,
  and `rmul_2025`, plus the matching world files: from
  `RoboMaster/Simulation/rmu_gazebo_simulator`, revision `41e6fe4`, Apache-2.0.
- `sentry_simulation/maps`: from `pb2025_sentry_nav`, revision
  `b5c6a5200a10bf9d7c6f1aa72358afcdfab84b05`, Apache-2.0.
- The point-cloud field conversion behavior is adapted from
  `pb2025_sentry_nav/ign_sim_pointcloud_tool`, and the world-to-body velocity
  conversion follows `pb2025_sentry_nav/fake_vel_transform`, at the same
  revision above, Apache-2.0.
- `sentry_simulation/plugins/mecanum_drive2`: from RoboMaster-OSS
  `rmoss_gz_plugins`, Apache-2.0. Its source files retain their copyright and
  license headers.

The adapters, launch files, simplified sentry models, and configuration added
in this module are part of this repository and use Apache-2.0.
