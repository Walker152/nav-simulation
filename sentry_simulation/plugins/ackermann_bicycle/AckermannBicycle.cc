#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include <gz/common/Console.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/msgs/twist.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

namespace sentry_simulation
{
// Ideal steering actuator with physical wheel contact and body dynamics.
// This owns only commands; existing publishers own groundtruth and joints.
class AckermannBicycle final : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  void Configure(const gz::sim::Entity & entity,
    const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager & ecm,
    gz::sim::EventManager &) override
  {
    gz::sim::Model model(entity);
    const std::array<std::string, 4> joint_keys{
      "left_steering_joint", "right_steering_joint", "left_joint", "right_joint"};
    for (const auto & key : {"wheel_base", "wheel_separation", "kingpin_width",
        "wheel_radius", "steering_limit", "topic"})
    {
      if (!sdf->HasElement(key))
      {
        gzerr << "AckermannBicycle requires " << key << std::endl;
        return;
      }
    }
    if (!model.Valid(ecm))
    {
      gzerr << "AckermannBicycle requires a model entity" << std::endl;
      return;
    }
    for (std::size_t i = 0; i < joints.size(); ++i)
    {
      if (!sdf->HasElement(joint_keys[i]))
      {
        gzerr << "AckermannBicycle requires " << joint_keys[i] << std::endl;
        return;
      }
      joints[i] = model.JointByName(ecm, sdf->Get<std::string>(joint_keys[i]));
      if (joints[i] == gz::sim::kNullEntity ||
        std::find(joints.begin(), joints.begin() + i, joints[i]) != joints.begin() + i)
      {
        gzerr << "AckermannBicycle requires four distinct existing joints" << std::endl;
        return;
      }
    }
    wheelbase = sdf->Get<double>("wheel_base");
    track = sdf->Get<double>("wheel_separation");
    kingpin = sdf->Get<double>("kingpin_width");
    radius = sdf->Get<double>("wheel_radius");
    const double limit = sdf->Get<double>("steering_limit");
    for (const double value : {wheelbase, track, kingpin, radius, limit})
    {
      if (!std::isfinite(value) || value <= 0)
      {
        gzerr << "AckermannBicycle geometry must be finite and positive" << std::endl;
        return;
      }
    }
    if (limit >= std::atan2(wheelbase, kingpin / 2))
    {
      gzerr << "AckermannBicycle steering limit crosses the inner kingpin" << std::endl;
      return;
    }
    max_curvature = std::tan(limit) / wheelbase;
    if (!std::isfinite(max_curvature) || !std::isfinite(track * max_curvature))
    {
      gzerr << "AckermannBicycle geometry overflows" << std::endl;
      return;
    }
    configured = node.Subscribe(sdf->Get<std::string>("topic"),
      &AckermannBicycle::OnCommand, this);
    if (!configured)
      gzerr << "AckermannBicycle could not subscribe to its command topic" << std::endl;
  }

  void PreUpdate(const gz::sim::UpdateInfo & info,
    gz::sim::EntityComponentManager & ecm) override
  {
    if (!configured)
      return;
    double v, w;
    {
      std::lock_guard<std::mutex> lock(command_mutex);
      if (info.dt < std::chrono::steady_clock::duration::zero())
        command_v = command_w = 0;
      v = command_v;
      w = command_w;
    }
    if (info.paused)
      return;
    // No yaw deadband: small finite commands obey the same signed curvature.
    double curvature = v == 0 ? 0 : std::clamp(w / v, -max_curvature, max_curvature);
    double left_rate = v * (1 - track * curvature / 2) / radius;
    double right_rate = v * (1 + track * curvature / 2) / radius;
    if (!std::isfinite(left_rate) || !std::isfinite(right_rate))
      curvature = left_rate = right_rate = 0;
    const std::array<double, 2> angles{
      std::atan(wheelbase * curvature / (1 - kingpin * curvature / 2)),
      std::atan(wheelbase * curvature / (1 + kingpin * curvature / 2))};
    for (std::size_t i = 0; i < angles.size(); ++i)
    {
      gz::sim::Joint steering(joints[i]);
      // Reset intentionally bypasses steering slew/effort. Zero velocity holds
      // the new configuration during physics; front rolling joints stay free.
      steering.ResetPosition(ecm, {angles[i]});
      steering.SetVelocity(ecm, {0});
    }
    gz::sim::Joint(joints[2]).SetVelocity(ecm, {left_rate});
    gz::sim::Joint(joints[3]).SetVelocity(ecm, {right_rate});
  }

private:
  void OnCommand(const gz::msgs::Twist & message)
  {
    std::lock_guard<std::mutex> lock(command_mutex);
    // At the center, body vx equals rear-axle speed. Center vy=d*w is a
    // consequence of rigid-body rotation, not an independently driven input.
    const double v = message.linear().x();
    const double w = message.angular().z();
    const bool valid = std::isfinite(v) && std::isfinite(w);
    command_v = valid ? v : 0;
    command_w = valid ? w : 0;
  }

  std::array<gz::sim::Entity, 4> joints{};
  double wheelbase{0}, track{0}, kingpin{0}, radius{0}, max_curvature{0};
  bool configured{false};
  std::mutex command_mutex;
  double command_v{0}, command_w{0};
  // Destroy transport before the callback's mutex and command state.
  gz::transport::Node node;
};
}  // namespace sentry_simulation

GZ_ADD_PLUGIN(sentry_simulation::AckermannBicycle,
  gz::sim::System, gz::sim::ISystemConfigure,
  gz::sim::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(sentry_simulation::AckermannBicycle,
  "sentry_simulation::AckermannBicycle")
