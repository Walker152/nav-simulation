#include "sentry_simulation/pointcloud_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "rclcpp_components/register_node_macro.hpp"

namespace
{

struct EIGEN_ALIGN16 PointXYZIRT
{
  PCL_ADD_POINT4D;
  float intensity;
  std::uint16_t ring;
  float time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace

POINT_CLOUD_REGISTER_POINT_STRUCT(
  PointXYZIRT,
  (float, x, x)(float, y, y)(float, z, z)
  (float, intensity, intensity)(std::uint16_t, ring, ring)(float, time, time))

namespace sentry_simulation
{

PointCloudAdapter::PointCloudAdapter(const rclcpp::NodeOptions & options)
: Node("pointcloud_adapter", options)
{
  const auto input_topic = declare_parameter<std::string>("input_topic", "/sim/lidar/points");
  const auto output_topic = declare_parameter<std::string>("output_topic", "/velodyne_points");
  n_scan_ = declare_parameter<int>("n_scan", 32);
  vertical_min_rad_ = declare_parameter<double>("vertical_min_deg", -7.0) * M_PI / 180.0;
  vertical_max_rad_ = declare_parameter<double>("vertical_max_deg", 52.0) * M_PI / 180.0;
  scan_period_ = declare_parameter<double>("scan_period", 0.05);

  publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    output_topic, rclcpp::SensorDataQoS());
  subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic, rclcpp::SensorDataQoS(),
    std::bind(&PointCloudAdapter::cloud_callback, this, std::placeholders::_1));
}

void PointCloudAdapter::cloud_callback(sensor_msgs::msg::PointCloud2::UniquePtr message)
{
  // Gazebo's packed GPU lidar cloud is only guaranteed to expose XYZ. Keep
  // the adapter independent of an optional upstream intensity field.
  pcl::PointCloud<pcl::PointXYZ> input;
  pcl::fromROSMsg(*message, input);

  pcl::PointCloud<PointXYZIRT> output;
  output.header = input.header;
  output.reserve(input.size());
  const double vertical_span = vertical_max_rad_ - vertical_min_rad_;

  const std::size_t horizontal_samples = input.width > 1 ? input.width : input.size();
  for (std::size_t index = 0; index < input.points.size(); ++index) {
    const auto & point = input.points[index];
    const double planar_range = std::hypot(point.x, point.y);
    const double vertical_angle = std::atan2(point.z, planar_range);
    const double normalized_ring =
      vertical_span > 0.0 ? (vertical_angle - vertical_min_rad_) / vertical_span : 0.0;
    const int ring = std::clamp(
      static_cast<int>(std::lround(normalized_ring * (n_scan_ - 1))), 0, n_scan_ - 1);

    PointXYZIRT converted;
    converted.x = point.x;
    converted.y = point.y;
    converted.z = point.z;
    converted.intensity = 0.0F;
    converted.ring = static_cast<std::uint16_t>(ring);
    const std::size_t horizontal_index = index % horizontal_samples;
    converted.time = horizontal_samples > 1 ?
      static_cast<float>(
      scan_period_ * static_cast<double>(horizontal_index) /
      static_cast<double>(horizontal_samples - 1)) : 0.0F;
    output.push_back(converted);
  }

  output.width = static_cast<std::uint32_t>(output.size());
  output.height = 1;
  output.is_dense = input.is_dense;
  auto ros_output = std::make_unique<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(output, *ros_output);
  ros_output->header = message->header;
  publisher_->publish(std::move(ros_output));
}

}  // namespace sentry_simulation

RCLCPP_COMPONENTS_REGISTER_NODE(sentry_simulation::PointCloudAdapter)
