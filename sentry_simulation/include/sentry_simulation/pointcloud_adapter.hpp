#ifndef SENTRY_SIMULATION__POINTCLOUD_ADAPTER_HPP_
#define SENTRY_SIMULATION__POINTCLOUD_ADAPTER_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace sentry_simulation
{

class PointCloudAdapter : public rclcpp::Node
{
public:
  explicit PointCloudAdapter(const rclcpp::NodeOptions & options);

private:
  void cloud_callback(sensor_msgs::msg::PointCloud2::UniquePtr message);

  int n_scan_;
  double vertical_min_rad_;
  double vertical_max_rad_;
  double scan_period_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

}  // namespace sentry_simulation

#endif  // SENTRY_SIMULATION__POINTCLOUD_ADAPTER_HPP_

