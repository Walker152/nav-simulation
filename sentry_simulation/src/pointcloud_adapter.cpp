#include "sentry_simulation/pointcloud_adapter.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <functional>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "rclcpp_components/register_node_macro.hpp"

namespace {
constexpr double kDegreesToRadians = M_PI / 180.0;
constexpr double kHalfPi = M_PI / 2.0;

}  // namespace

namespace sentry_simulation {

PointCloudAdapter::SensorTransform PointCloudAdapter::make_sensor_transform(
  const std::vector<double> & pose)
{
  if (pose.size() != 6) {
    throw std::invalid_argument("MID360 sensor pose must contain x, y, z, roll, pitch, yaw");
  }
  const double cr = std::cos(pose[3]);
  const double sr = std::sin(pose[3]);
  const double cp = std::cos(pose[4]);
  const double sp = std::sin(pose[4]);
  const double cy = std::cos(pose[5]);
  const double sy = std::sin(pose[5]);
  return SensorTransform{pose[0],
    pose[1],
    pose[2],
    cy * cp,
    cy * sp * sr - sy * cr,
    cy * sp * cr + sy * sr,
    sy * cp,
    sy * sp * sr + cy * cr,
    sy * sp * cr - cy * sr,
    -sp,
    cp * sr,
    cp * cr};
}

void PointCloudAdapter::transform_point(const SensorTransform & transform,
  double input_x,
  double input_y,
  double input_z,
  float & output_x,
  float & output_y,
  float & output_z)
{
  output_x = static_cast<float>(
    transform.r00 * input_x + transform.r01 * input_y + transform.r02 * input_z + transform.x);
  output_y = static_cast<float>(
    transform.r10 * input_x + transform.r11 * input_y + transform.r12 * input_z + transform.y);
  output_z = static_cast<float>(
    transform.r20 * input_x + transform.r21 * input_y + transform.r22 * input_z + transform.z);
}

PointCloudAdapter::PointCloudAdapter(const rclcpp::NodeOptions & options)
: Node("pointcloud_adapter", options)
{
  const auto left_front_input_topic =
    declare_parameter<std::string>("left_front_input_topic", "/sim/lidar/left/front/points");
  const auto left_rear_input_topic =
    declare_parameter<std::string>("left_rear_input_topic", "/sim/lidar/left/rear/points");
  const auto right_front_input_topic =
    declare_parameter<std::string>("right_front_input_topic", "/sim/lidar/right/front/points");
  const auto right_rear_input_topic =
    declare_parameter<std::string>("right_rear_input_topic", "/sim/lidar/right/rear/points");
  const auto output_topic = declare_parameter<std::string>("output_topic", "/livox/lidar");
  output_frame_id_ = declare_parameter<std::string>("output_frame_id", "sim_lidar");
  const auto pattern_file = declare_parameter<std::string>("pattern_file");
  pattern_points_per_frame_ = declare_parameter<int>("pattern_points_per_frame", 20000);
  vertical_min_rad_ = declare_parameter<double>("vertical_min_deg", -7.3) * kDegreesToRadians;
  vertical_max_rad_ = declare_parameter<double>("vertical_max_deg", 52.3) * kDegreesToRadians;
  scan_period_ = declare_parameter<double>("scan_period", 0.1);
  const double sync_tolerance_ms = declare_parameter<double>("sync_tolerance_ms", 5.0);
  sync_tolerance_ns_ = static_cast<std::int64_t>(std::llround(sync_tolerance_ms * 1.0e6));
  left_transform_ = make_sensor_transform(declare_parameter<std::vector<double>>(
    "left_sensor_pose", {-0.0496, 0.352530918687, 0.0, -0.5835987756, 0.0, 0.0}));
  right_transform_ = make_sensor_transform(declare_parameter<std::vector<double>>(
    "right_sensor_pose", {-0.0496, 0.047469081313, 0.0, 0.5835987756, 0.0, 0.0}));

  if (pattern_points_per_frame_ <= 0 || scan_period_ <= 0.0 || sync_tolerance_ns_ < 0 ||
      vertical_max_rad_ <= vertical_min_rad_ || output_frame_id_.empty()) {
    throw std::invalid_argument("invalid MID360 scan pattern parameters");
  }

  std::ifstream pattern_stream(pattern_file);
  if (!pattern_stream) {
    throw std::runtime_error("cannot open MID360 scan pattern: " + pattern_file);
  }
  pattern_.reserve(800000);
  std::string line;
  while (std::getline(pattern_stream, line)) {
    std::istringstream row(line);
    std::string time_text;
    std::string azimuth_text;
    std::string zenith_text;
    if (!std::getline(row, time_text, ',') || !std::getline(row, azimuth_text, ',') ||
        !std::getline(row, zenith_text, ',')) {
      continue;
    }
    try {
      PatternRay ray;
      ray.azimuth_rad = std::stod(azimuth_text) * kDegreesToRadians;
      ray.elevation_rad = (90.0 - std::stod(zenith_text)) * kDegreesToRadians;
      pattern_.push_back(ray);
    } catch (const std::invalid_argument &) {
      // Header row.
    } catch (const std::out_of_range &) {
      throw std::runtime_error("out-of-range value in MID360 scan pattern: " + pattern_file);
    }
  }
  if (pattern_.empty()) {
    throw std::runtime_error("MID360 scan pattern contains no rays: " + pattern_file);
  }

  RCLCPP_INFO(get_logger(),
    "Loaded %zu MID360 pattern rays; publishing up to %d points per dual-lidar frame",
    pattern_.size(),
    pattern_points_per_frame_ * 2);
  publisher_ = create_publisher<livox_ros_driver2::msg::CustomMsg>(output_topic, rclcpp::SensorDataQoS());
  left_front_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(left_front_input_topic,
    rclcpp::SensorDataQoS(),
    std::bind(&PointCloudAdapter::left_front_cloud_callback, this, std::placeholders::_1));
  left_rear_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(left_rear_input_topic,
    rclcpp::SensorDataQoS(),
    std::bind(&PointCloudAdapter::left_rear_cloud_callback, this, std::placeholders::_1));
  right_front_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(right_front_input_topic,
    rclcpp::SensorDataQoS(),
    std::bind(&PointCloudAdapter::right_front_cloud_callback, this, std::placeholders::_1));
  right_rear_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(right_rear_input_topic,
    rclcpp::SensorDataQoS(),
    std::bind(&PointCloudAdapter::right_rear_cloud_callback, this, std::placeholders::_1));
}

void PointCloudAdapter::left_front_cloud_callback(sensor_msgs::msg::PointCloud2::UniquePtr message)
{
  left_front_cloud_ = std::move(message);
  publish_synchronized_clouds();
}

void PointCloudAdapter::left_rear_cloud_callback(sensor_msgs::msg::PointCloud2::UniquePtr message)
{
  left_rear_cloud_ = std::move(message);
  publish_synchronized_clouds();
}

void PointCloudAdapter::right_front_cloud_callback(sensor_msgs::msg::PointCloud2::UniquePtr message)
{
  right_front_cloud_ = std::move(message);
  publish_synchronized_clouds();
}

void PointCloudAdapter::right_rear_cloud_callback(sensor_msgs::msg::PointCloud2::UniquePtr message)
{
  right_rear_cloud_ = std::move(message);
  publish_synchronized_clouds();
}

void PointCloudAdapter::reset_clouds()
{
  left_front_cloud_.reset();
  left_rear_cloud_.reset();
  right_front_cloud_.reset();
  right_rear_cloud_.reset();
}

void PointCloudAdapter::publish_synchronized_clouds()
{
  if (!left_front_cloud_ || !left_rear_cloud_ || !right_front_cloud_ || !right_rear_cloud_) {
    return;
  }

  const std::array<std::int64_t, 4> stamps{rclcpp::Time(left_front_cloud_->header.stamp).nanoseconds(),
    rclcpp::Time(left_rear_cloud_->header.stamp).nanoseconds(),
    rclcpp::Time(right_front_cloud_->header.stamp).nanoseconds(),
    rclcpp::Time(right_rear_cloud_->header.stamp).nanoseconds()};
  const auto [min_stamp, max_stamp] = std::minmax_element(stamps.begin(), stamps.end());
  if (*max_stamp - *min_stamp > sync_tolerance_ns_) {
    if (stamps[0] == *min_stamp) {
      left_front_cloud_.reset();
    }
    if (stamps[1] == *min_stamp) {
      left_rear_cloud_.reset();
    }
    if (stamps[2] == *min_stamp) {
      right_front_cloud_.reset();
    }
    if (stamps[3] == *min_stamp) {
      right_rear_cloud_.reset();
    }
    return;
  }

  pcl::PointCloud<pcl::PointXYZ> left_front_input;
  pcl::PointCloud<pcl::PointXYZ> left_rear_input;
  pcl::PointCloud<pcl::PointXYZ> right_front_input;
  pcl::PointCloud<pcl::PointXYZ> right_rear_input;
  pcl::fromROSMsg(*left_front_cloud_, left_front_input);
  pcl::fromROSMsg(*left_rear_cloud_, left_rear_input);
  pcl::fromROSMsg(*right_front_cloud_, right_front_input);
  pcl::fromROSMsg(*right_rear_cloud_, right_rear_input);

  const auto is_organized = [](const pcl::PointCloud<pcl::PointXYZ> & cloud) {
    return cloud.width >= 2 && cloud.height >= 2 &&
           static_cast<std::size_t>(cloud.width) * cloud.height == cloud.points.size();
  };
  const auto pair_matches = [&](const pcl::PointCloud<pcl::PointXYZ> & front,
                              const pcl::PointCloud<pcl::PointXYZ> & rear) {
    return is_organized(front) && is_organized(rear) && front.width == rear.width &&
           front.height == rear.height;
  };
  if (!pair_matches(left_front_input, left_rear_input) ||
      !pair_matches(right_front_input, right_rear_input)) {
    RCLCPP_WARN_THROTTLE(get_logger(),
      *get_clock(),
      5000,
      "Dual MID360 adapter requires organized matching front/rear clouds");
    reset_clouds();
    return;
  }

  auto output = std::make_unique<livox_ros_driver2::msg::CustomMsg>();
  const rclcpp::Time snapshot_stamp(*max_stamp, RCL_ROS_TIME);
  constexpr std::uint32_t snapshot_offset_time = 0;
  output->header = left_front_cloud_->header;
  output->header.stamp = snapshot_stamp;
  output->header.frame_id = output_frame_id_;
  output->timebase = static_cast<std::uint64_t>(snapshot_stamp.nanoseconds());
  output->lidar_id = 0;
  output->points.reserve(static_cast<std::size_t>(pattern_points_per_frame_) * 2);
  const double vertical_span = vertical_max_rad_ - vertical_min_rad_;

  const auto append_sensor_point = [&](const pcl::PointCloud<pcl::PointXYZ> & front,
                                     const pcl::PointCloud<pcl::PointXYZ> & rear,
                                     const SensorTransform & sensor_transform,
                                     const PatternRay & ray,
                                     std::size_t ray_index,
                                     std::uint32_t offset_time) {
    const bool use_rear = ray.azimuth_rad > kHalfPi || ray.azimuth_rad < -kHalfPi;
    double local_azimuth = ray.azimuth_rad;
    if (use_rear) {
      local_azimuth += ray.azimuth_rad > 0.0 ? -M_PI : M_PI;
    }
    const auto & input = use_rear ? rear : front;
    const double horizontal_position = std::clamp((local_azimuth + kHalfPi) / M_PI, 0.0, 1.0);
    const double vertical_position =
      std::clamp((ray.elevation_rad - vertical_min_rad_) / vertical_span, 0.0, 1.0);
    const std::size_t column =
      static_cast<std::size_t>(std::lround(horizontal_position * static_cast<double>(input.width - 1)));
    const std::size_t row =
      static_cast<std::size_t>(std::lround(vertical_position * static_cast<double>(input.height - 1)));
    const auto & point = input.points[row * input.width + column];
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      return;
    }

    const double sensor_x = use_rear ? -point.x : point.x;
    const double sensor_y = use_rear ? -point.y : point.y;
    livox_ros_driver2::msg::CustomPoint converted;
    transform_point(sensor_transform, sensor_x, sensor_y, point.z, converted.x, converted.y, converted.z);
    converted.reflectivity = 0;
    converted.tag = 0x10;
    converted.line = static_cast<std::uint8_t>(ray_index % 4);
    converted.offset_time = offset_time;
    output->points.push_back(converted);
  };

  for (int offset = 0; offset < pattern_points_per_frame_; ++offset) {
    const std::size_t ray_index = (pattern_index_ + static_cast<std::size_t>(offset)) % pattern_.size();
    const auto & ray = pattern_[ray_index];
    append_sensor_point(
      left_front_input, left_rear_input, left_transform_, ray, ray_index, snapshot_offset_time);
    append_sensor_point(
      right_front_input, right_rear_input, right_transform_, ray, ray_index, snapshot_offset_time);
  }
  pattern_index_ = (pattern_index_ + static_cast<std::size_t>(pattern_points_per_frame_)) % pattern_.size();

  output->point_num = static_cast<std::uint32_t>(output->points.size());
  publisher_->publish(std::move(output));
  reset_clouds();
}

}  // namespace sentry_simulation

RCLCPP_COMPONENTS_REGISTER_NODE(sentry_simulation::PointCloudAdapter)
