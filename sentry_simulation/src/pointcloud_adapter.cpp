#include "sentry_simulation/pointcloud_adapter.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <functional>
#include <memory>
#include <limits>
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

Eigen::Isometry3d PointCloudAdapter::make_sensor_transform(const std::vector<double> & pose)
{
  if (pose.size() != 6 || !std::all_of(pose.begin(), pose.end(),
      [](double value) {return std::isfinite(value);})) {
    throw std::invalid_argument("sensor pose must contain finite x, y, z, roll, pitch, yaw");
  }
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = Eigen::Vector3d(pose[0], pose[1], pose[2]);
  result.linear() = (Eigen::AngleAxisd(pose[5], Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(pose[4], Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(pose[3], Eigen::Vector3d::UnitX())).toRotationMatrix();
  return result;
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
  const double scan_period = declare_parameter<double>("scan_period", 0.1);
  const double sync_tolerance_ms = declare_parameter<double>("sync_tolerance_ms", 5.0);
  if (!std::isfinite(scan_period) || scan_period < 1.0e-9 ||
      scan_period * 1.0e9 > std::numeric_limits<std::uint32_t>::max() ||
      !std::isfinite(sync_tolerance_ms) || sync_tolerance_ms < 0.0 ||
      sync_tolerance_ms > scan_period * 1.0e3) {
    throw std::invalid_argument("invalid scan period or synchronization tolerance");
  }
  scan_period_ns_ = static_cast<std::int64_t>(std::llround(scan_period * 1.0e9));
  sync_tolerance_ns_ = static_cast<std::int64_t>(std::llround(sync_tolerance_ms * 1.0e6));
  // OdometryPublisher reports the model origin, irrespective of its child-frame label.
  lidar_pose_in_model_ = make_sensor_transform(declare_parameter<std::vector<double>>(
    "lidar_pose_in_model", {0.0, -0.2, 0.3, 0.0, 0.0, 0.0}));
  left_transform_ = make_sensor_transform(declare_parameter<std::vector<double>>(
    "left_sensor_pose", {-0.0496, 0.352530918687, 0.0, -0.5835987756, 0.0, 0.0}));
  right_transform_ = make_sensor_transform(declare_parameter<std::vector<double>>(
    "right_sensor_pose", {-0.0496, 0.047469081313, 0.0, 0.5835987756, 0.0, 0.0}));

  if (pattern_points_per_frame_ <= 0 ||
      pattern_points_per_frame_ > std::numeric_limits<int>::max() / 2 ||
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
  ground_truth_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
    declare_parameter<std::string>("ground_truth_topic", "/sim/ground_truth/odom"),
    rclcpp::SensorDataQoS(),
    std::bind(&PointCloudAdapter::ground_truth_callback, this, std::placeholders::_1));
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

void PointCloudAdapter::ground_truth_callback(nav_msgs::msg::Odometry::ConstSharedPtr message)
{
  const auto & p = message->pose.pose.position;
  const auto & q = message->pose.pose.orientation;
  if (message->header.stamp.sec < 0 || message->header.stamp.nanosec >= 1000000000U) {return;}
  TimedPose pose{rclcpp::Time(message->header.stamp).nanoseconds(),
    Eigen::Vector3d(p.x, p.y, p.z), Eigen::Quaterniond(q.w, q.x, q.y, q.z)};
  if (!pose.position.allFinite() || !pose.rotation.coeffs().allFinite() ||
      !std::isfinite(pose.rotation.norm()) || pose.rotation.norm() < 1.0e-6) {
    return;
  }
  pose.rotation.normalize();
  if (!pose_history_.empty() && pose.stamp < pose_history_.back().stamp) {
    pose_history_.clear();
    reset_clouds();
    last_scan_end_ = -1;
  }
  if (!pose_history_.empty() && pose.stamp == pose_history_.back().stamp) {
    pose_history_.back() = pose;
  } else {
    pose_history_.push_back(pose);
  }
  const auto history_span = std::max<std::int64_t>(1000000000, 2 * scan_period_ns_);
  while (pose_history_.size() > 2 &&
      (pose_history_[1].stamp < pose.stamp - history_span || pose_history_.size() > 4096)) {
    pose_history_.pop_front();
  }
  publish_synchronized_clouds();
}

Eigen::Isometry3d PointCloudAdapter::lidar_pose_at(std::int64_t stamp) const
{
  // publish_synchronized_clouds has checked history coverage and interpolation gaps.
  const auto upper = std::lower_bound(pose_history_.begin(), pose_history_.end(), stamp,
    [](const TimedPose & pose, std::int64_t time) {return pose.stamp < time;});
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  if (upper->stamp == stamp) {
    result.translation() = upper->position;
    result.linear() = upper->rotation.toRotationMatrix();
  } else {
    const auto lower = std::prev(upper);
    const double alpha = static_cast<double>(stamp - lower->stamp) /
      static_cast<double>(upper->stamp - lower->stamp);
    result.translation() = (1.0 - alpha) * lower->position + alpha * upper->position;
    result.linear() = lower->rotation.slerp(alpha, upper->rotation).toRotationMatrix();
  }
  return result * lidar_pose_in_model_;
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

  const std::int64_t scan_end = *max_stamp;
  const std::int64_t scan_start = std::max(scan_end - scan_period_ns_, last_scan_end_);
  if (scan_start < 0 || scan_end <= last_scan_end_) {
    reset_clouds();
    return;
  }
  if (pose_history_.empty()) {return;}
  const std::int64_t first_pose_time = std::min(scan_start, *min_stamp);
  if (pose_history_.front().stamp > first_pose_time) {
    reset_clouds();
    return;
  }
  if (pose_history_.back().stamp < scan_end) {return;}
  // The configured 100 Hz truth stream must not bridge long outages by interpolation.
  for (auto next = std::next(pose_history_.begin()); next != pose_history_.end(); ++next) {
    const auto previous = std::prev(next);
    if (next->stamp > first_pose_time && previous->stamp < scan_end &&
        next->stamp - previous->stamp > 50000000) {
      reset_clouds();
      return;
    }
  }
  const std::array<Eigen::Isometry3d, 4> source_poses{
    lidar_pose_at(stamps[0]), lidar_pose_at(stamps[1]),
    lidar_pose_at(stamps[2]), lidar_pose_at(stamps[3])};

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
  output->header = left_front_cloud_->header;
  output->header.stamp = rclcpp::Time(scan_start, RCL_ROS_TIME);
  output->header.frame_id = output_frame_id_;
  output->timebase = static_cast<std::uint64_t>(scan_start);
  output->lidar_id = 0;
  output->points.reserve(static_cast<std::size_t>(pattern_points_per_frame_) * 2);
  const double vertical_span = vertical_max_rad_ - vertical_min_rad_;

  for (int offset = 0; offset < pattern_points_per_frame_; ++offset) {
    const std::size_t ray_index = (pattern_index_ + static_cast<std::size_t>(offset)) % pattern_.size();
    const auto & ray = pattern_[ray_index];
    const auto offset_time = static_cast<std::uint32_t>(
      (scan_end - scan_start) * static_cast<std::int64_t>(offset) / pattern_points_per_frame_);
    const Eigen::Isometry3d point_from_world = lidar_pose_at(scan_start + offset_time).inverse();
    const bool use_rear = ray.azimuth_rad > kHalfPi || ray.azimuth_rad < -kHalfPi;
    double local_azimuth = ray.azimuth_rad;
    if (use_rear) {
      local_azimuth += ray.azimuth_rad > 0.0 ? -M_PI : M_PI;
    }
    const double horizontal_position = std::clamp((local_azimuth + kHalfPi) / M_PI, 0.0, 1.0);
    const double vertical_position =
      std::clamp((ray.elevation_rad - vertical_min_rad_) / vertical_span, 0.0, 1.0);
    const auto append_sensor_point = [&](const pcl::PointCloud<pcl::PointXYZ> & front,
                                       const pcl::PointCloud<pcl::PointXYZ> & rear,
                                       const Eigen::Isometry3d & sensor_transform,
                                       std::size_t source_index) {
      const auto & input = use_rear ? rear : front;
      const std::size_t column = static_cast<std::size_t>(std::lround(
        horizontal_position * static_cast<double>(input.width - 1)));
      const std::size_t row = static_cast<std::size_t>(std::lround(
        vertical_position * static_cast<double>(input.height - 1)));
      const auto & point = input.points[row * input.width + column];
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        return;
      }
      const Eigen::Vector3d sensor_point(
        use_rear ? -point.x : point.x, use_rear ? -point.y : point.y, point.z);
      // Re-express the same static world surface at its assigned scan time.
      // Snapshot visibility and dynamic objects are not reconstructed.
      const Eigen::Vector3d timed_point = point_from_world *
        (source_poses[source_index + (use_rear ? 1 : 0)] * (sensor_transform * sensor_point));
      livox_ros_driver2::msg::CustomPoint converted;
      converted.x = static_cast<float>(timed_point.x());
      converted.y = static_cast<float>(timed_point.y());
      converted.z = static_cast<float>(timed_point.z());
      converted.reflectivity = 0;
      converted.tag = 0x10;
      converted.line = static_cast<std::uint8_t>(ray_index % 4);
      converted.offset_time = offset_time;
      output->points.push_back(converted);
    };
    append_sensor_point(left_front_input, left_rear_input, left_transform_, 0);
    append_sensor_point(right_front_input, right_rear_input, right_transform_, 2);
  }
  pattern_index_ = (pattern_index_ + static_cast<std::size_t>(pattern_points_per_frame_)) % pattern_.size();

  output->point_num = static_cast<std::uint32_t>(output->points.size());
  publisher_->publish(std::move(output));
  last_scan_end_ = scan_end;
  reset_clouds();
}

}  // namespace sentry_simulation

RCLCPP_COMPONENTS_REGISTER_NODE(sentry_simulation::PointCloudAdapter)
