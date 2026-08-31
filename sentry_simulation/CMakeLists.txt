cmake_minimum_required(VERSION 3.8)
project(sentry_simulation)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-msgs8 REQUIRED)
find_package(ignition-transport11 REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(pointcloud_adapter_component SHARED src/pointcloud_adapter.cpp)
target_compile_features(pointcloud_adapter_component PUBLIC cxx_std_17)
target_include_directories(pointcloud_adapter_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS})
ament_target_dependencies(pointcloud_adapter_component
  livox_ros_driver2 pcl_conversions rclcpp rclcpp_components sensor_msgs)
target_link_libraries(pointcloud_adapter_component ${PCL_LIBRARIES})
rclcpp_components_register_nodes(
  pointcloud_adapter_component "sentry_simulation::PointCloudAdapter")

add_library(MecanumDrive2 SHARED
  plugins/mecanum_drive2/MecanumDrive2.cc)
target_compile_features(MecanumDrive2 PUBLIC cxx_std_17)
target_include_directories(MecanumDrive2 PRIVATE plugins/mecanum_drive2)
target_link_libraries(MecanumDrive2
  ignition-gazebo6::ignition-gazebo6)

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS sentry_simulation/cmd_vel_adapter.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME sentry_sim_cmd_adapter)
install(PROGRAMS sentry_simulation/imu_filter.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME sentry_sim_imu_filter)
install(DIRECTORY include/ DESTINATION include)
install(TARGETS pointcloud_adapter_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS MecanumDrive2 LIBRARY DESTINATION plugins)
install(DIRECTORY behavior_tree config launch maps resource
  DESTINATION share/${PROJECT_NAME})
install(FILES ../README.md ../THIRD_PARTY_NOTICES.md
  DESTINATION share/${PROJECT_NAME})

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/gazebo.dsv.in")

ament_export_include_directories(include)
ament_export_dependencies(livox_ros_driver2 pcl_conversions rclcpp rclcpp_components sensor_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(simulation_kinematics test/test_kinematics.py)
  ament_add_pytest_test(simulation_imu_filter test/test_imu_filter.py)
  ament_add_pytest_test(simulation_contract test/test_simulation_contract.py)
endif()

ament_package()
