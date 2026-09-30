#include "../gazebo_compat.hh"
#ifdef SENTRY_GAZEBO_FORTRESS
#include <ignition/msgs/boolean.pb.h>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/World.hh>
#else
#include <gz/msgs/boolean.pb.h>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/World.hh>
#endif

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <string>
#include <vector>

namespace sentry_simulation
{
// Prescribed-motion collision proxy, not a dynamic human body. Static models
// remain collidable; WorldPoseCmd moves their physics free group as well as Pose.
// Absolute world waypoints require a top-level model (Gazebo API restriction).
class ScriptedObstacle final : public SENTRY_GZ::SENTRY_SIM::System,
  public SENTRY_GZ::SENTRY_SIM::ISystemConfigure,
  public SENTRY_GZ::SENTRY_SIM::ISystemPreUpdate
#ifndef SENTRY_GAZEBO_FORTRESS
  , public SENTRY_GZ::SENTRY_SIM::ISystemReset
#endif
{
public:
  void Configure(const SENTRY_GZ::SENTRY_SIM::Entity &entity,
    const std::shared_ptr<const sdf::Element> &sdf,
    SENTRY_GZ::SENTRY_SIM::EntityComponentManager &ecm,
    SENTRY_GZ::SENTRY_SIM::EventManager &) override
  {
    model = SENTRY_GZ::SENTRY_SIM::Model(entity);
    if (!model.Valid(ecm) || !model.Static(ecm))
    {
      gzerr << "ScriptedObstacle requires a static model" << std::endl;
      return;
    }
    if (!sdf->HasElement("topic") || !sdf->HasElement("waypoint"))
    {
      gzerr << "ScriptedObstacle requires topic and closed-loop waypoints" << std::endl;
      return;
    }
    // Clone only configuration data; GetElement's API is mutable.
    auto config = sdf->Clone();
    for (auto item = config->GetElement("waypoint"); item;
      item = item->GetNextElement("waypoint"))
    {
      if (!item->HasElement("time") || !item->HasElement("pose"))
      {
        gzerr << "ScriptedObstacle waypoint requires time and pose" << std::endl;
        return;
      }
      const double time = item->Get<double>("time");
      const auto pose = item->Get<SENTRY_GZ::math::Pose3d>("pose");
      if (!std::isfinite(time) || time < 0 || !pose.Pos().IsFinite() ||
        !pose.Rot().IsFinite() || (!waypoints.empty() && time <= waypoints.back().time))
      {
        gzerr << "ScriptedObstacle waypoints must be finite with increasing time" << std::endl;
        return;
      }
      waypoints.push_back({time, pose});
    }
    if (waypoints.size() < 2 || waypoints.front().time != 0 ||
      waypoints.front().pose != waypoints.back().pose)
    {
      gzerr << "ScriptedObstacle needs t=0 and an identical closing pose" << std::endl;
      return;
    }
    initial_enabled = sdf->Get<bool>("enabled", true).first;
    enabled.store(initial_enabled);
    configured = node.Subscribe(sdf->Get<std::string>("topic"),
      &ScriptedObstacle::OnEnabled, this);
    if (!configured)
      gzerr << "ScriptedObstacle could not subscribe to its enabled topic" << std::endl;
  }

  void PreUpdate(const SENTRY_GZ::SENTRY_SIM::UpdateInfo &info,
    SENTRY_GZ::SENTRY_SIM::EntityComponentManager &ecm) override
  {
    if (!configured)
      return;
    // Gazebo loads model plugins before attaching the world ParentEntity.
    if (!hierarchy_checked)
    {
      const auto parent = ecm.Component<SENTRY_GZ::SENTRY_SIM::components::ParentEntity>(model.Entity());
      if (!parent || !ecm.Component<SENTRY_GZ::SENTRY_SIM::components::World>(parent->Data()))
      {
        gzerr << "ScriptedObstacle requires a top-level model" << std::endl;
        configured = false;
        return;
      }
      hierarchy_checked = true;
    }
    if (info.dt < std::chrono::steady_clock::duration::zero())
      ResetMotion(ecm);
    if (info.paused)
      return;
    const double dt = std::chrono::duration<double>(info.dt).count();
    if (enabled.load() && dt > 0)
      phase = std::fmod(phase + dt, waypoints.back().time);
    auto next = std::upper_bound(waypoints.begin(), waypoints.end(), phase,
      [](double value, const Waypoint &point) { return value < point.time; });
    const auto &a = *(next - 1);
    const auto &b = *next;
    const double ratio = (phase - a.time) / (b.time - a.time);
    model.SetWorldPoseCmd(ecm, SENTRY_GZ::math::Pose3d(
      a.pose.Pos() + ratio * (b.pose.Pos() - a.pose.Pos()),
      SENTRY_GZ::math::Quaterniond::Slerp(ratio, a.pose.Rot(), b.pose.Rot())));
  }

#ifndef SENTRY_GAZEBO_FORTRESS
  void Reset(const SENTRY_GZ::SENTRY_SIM::UpdateInfo &,
    SENTRY_GZ::SENTRY_SIM::EntityComponentManager &ecm) override
  {
    if (configured)
      ResetMotion(ecm);
  }
#endif

private:
  void ResetMotion(SENTRY_GZ::SENTRY_SIM::EntityComponentManager &ecm)
  {
    phase = 0;
    enabled.store(initial_enabled);
    model.SetWorldPoseCmd(ecm, waypoints.front().pose);
  }

  void OnEnabled(const SENTRY_GZ::msgs::Boolean &message)
  {
    enabled.store(message.data());
  }

  struct Waypoint
  {
    double time;
    SENTRY_GZ::math::Pose3d pose;
  };
  SENTRY_GZ::SENTRY_SIM::Model model{SENTRY_GZ::SENTRY_SIM::kNullEntity};
  std::vector<Waypoint> waypoints;
  double phase{0};
  bool configured{false};
  bool hierarchy_checked{false};
  bool initial_enabled{true};
  std::atomic<bool> enabled{true};
  // Destroy transport before state accessed by its callback.
  SENTRY_GZ::transport::Node node;
};
}  // namespace sentry_simulation

#ifdef SENTRY_GAZEBO_FORTRESS
GZ_ADD_PLUGIN(sentry_simulation::ScriptedObstacle,
  SENTRY_GZ::SENTRY_SIM::System, SENTRY_GZ::SENTRY_SIM::ISystemConfigure,
  SENTRY_GZ::SENTRY_SIM::ISystemPreUpdate)
#else
GZ_ADD_PLUGIN(sentry_simulation::ScriptedObstacle,
  SENTRY_GZ::SENTRY_SIM::System, SENTRY_GZ::SENTRY_SIM::ISystemConfigure,
  SENTRY_GZ::SENTRY_SIM::ISystemPreUpdate, SENTRY_GZ::SENTRY_SIM::ISystemReset)
#endif
GZ_ADD_PLUGIN_ALIAS(sentry_simulation::ScriptedObstacle,
  "sentry_simulation::ScriptedObstacle")
