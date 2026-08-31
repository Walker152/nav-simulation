#ifndef SENTRY_SIMULATION__POINTCLOUD_ADAPTER_HPP_
#define SENTRY_SIMULATION__POINTCLOUD_ADAPTER_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace sentry_simulation {

class PointCloudAdapter : public rclcpp::Node
{
public:
  explicit PointCloudAdapter(const rclcpp::NodeOptions & options);

private:
  struct PatternRay
  {
    double azimuth_rad;
    double elevation_rad;
  };

  struct SensorTransform
  {
    double x;
    double y;
    double z;
    double r00;
    double r01;
    double r02;
    double r10;
    double r11;
    double r12;
    double r20;
    double r21;
    double r22;
  };

  static SensorTransform make_sensor_transform(const std::vector<double> & pose);
  static void transform_point(const SensorTransform & transform,
    double input_x,
    double input_y,
    double input_z,
    float & output_x,
    float & output_y,
    float & output_z);

  void left_front_cloud_callback(sensor_msgs::msg::PointCloud2::UniquePtr message);
  void left_rear_cloud_callback(sensor_msgs::msg::PointCloud2::UniquePtr message);
  void right_front_cloud_callback(sensor_msgs::msg::PointCloud2::UniquePtr message);
  void right_rear_cloud_callback(sensor_msgs::msg::PointCloud2::UniquePtr message);
  void publish_synchronized_clouds();
  void reset_clouds();

  std::vector<PatternRay> pattern_;
  std::size_t pattern_index_{0};
  int pattern_points_per_frame_;
  double vertical_min_rad_;
  double vertical_max_rad_;
  double scan_period_;
  std::int64_t sync_tolerance_ns_;
  std::string output_frame_id_;
  SensorTransform left_transform_;
  SensorTransform right_transform_;
  sensor_msgs::msg::PointCloud2::UniquePtr left_front_cloud_;
  sensor_msgs::msg::PointCloud2::UniquePtr left_rear_cloud_;
  sensor_msgs::msg::PointCloud2::UniquePtr right_front_cloud_;
  sensor_msgs::msg::PointCloud2::UniquePtr right_rear_cloud_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr left_front_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr left_rear_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr right_front_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr right_rear_subscription_;
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr publisher_;
};

}  // namespace sentry_simulation

#endif  // SENTRY_SIMULATION__POINTCLOUD_ADAPTER_HPP_
