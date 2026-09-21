#include "SwerveControl.hpp"
#include <chrono>
#include <memory>
#include <mutex>
#include <ignition/common/Console.hh>
#include <ignition/gazebo/Joint.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/msgs/model.pb.h>
#include <ignition/msgs/odometry.pb.h>
#include <ignition/msgs/twist.pb.h>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

namespace sentry_simulation {
namespace {
const std::array<std::string,4> prefixes{"front_left","front_right","rear_left","rear_right"};
void header(ignition::msgs::Header * h, double now, const std::string & frame) {
  const auto ns=static_cast<int64_t>(std::llround(now*1e9));
  h->mutable_stamp()->set_sec(ns/1000000000);
  h->mutable_stamp()->set_nsec(ns%1000000000);
  auto * data=h->add_data(); data->set_key("frame_id"); data->add_value(frame);
}
}
// Sole owner of actuator commands. Joint forces feed Gazebo physics; measured
// positions/velocities feed the servo, published states, and wheel odometry.
class SwerveDrive final : public ignition::gazebo::System,
  public ignition::gazebo::ISystemConfigure,
  public ignition::gazebo::ISystemPreUpdate,
  public ignition::gazebo::ISystemPostUpdate {
public:
  void Configure(const ignition::gazebo::Entity & entity,
    const std::shared_ptr<const sdf::Element> & sdf,
    ignition::gazebo::EntityComponentManager & ecm,
    ignition::gazebo::EventManager &) override {
    try {
      // All fields are explicit in generated SDF; no unread YAML-only limits.
      const std::pair<const char *,double *> fields[]{
        {"wheel_radius",&p.wheel_radius},{"wheelbase",&p.wheelbase},{"track",&p.track},
        {"steering_limit",&p.steering_limit},{"steering_hard_limit",&p.steering_hard_limit},
        {"max_linear_speed",&p.max_linear_speed},{"max_yaw_speed",&p.max_yaw_speed},
        {"max_wheel_speed",&p.max_wheel_speed},{"linear_acceleration",&p.linear_acceleration},
        {"yaw_acceleration",&p.yaw_acceleration},{"max_steering_velocity",&p.max_steering_velocity},
        {"max_steering_acceleration",&p.max_steering_acceleration},
        {"command_timeout",&p.command_timeout},{"publish_rate",&p.publish_rate},
        {"steering_kp",&p.steering_kp},{"steering_kd",&p.steering_kd},
        {"steering_torque",&p.steering_torque},{"wheel_kp",&p.wheel_kp},{"wheel_torque",&p.wheel_torque}};
      for (const auto & field:fields) {
        if (!sdf->HasElement(field.first)) throw std::invalid_argument(std::string("missing ")+field.first);
        *field.second=sdf->Get<double>(field.first);
      }
      control=std::make_unique<swerve::Control>(p);
      ignition::gazebo::Model model(entity);
      if (!model.Valid(ecm)) throw std::invalid_argument("requires a model entity");
      for (std::size_t i=0;i<4;++i) {
        steering[i]=model.JointByName(ecm,prefixes[i]+"_steer_joint");
        wheels[i]=model.JointByName(ecm,prefixes[i]+"_wheel_joint");
        for (auto joint:{steering[i],wheels[i]}) {
          if (joint==ignition::gazebo::kNullEntity) throw std::invalid_argument("missing swerve joint");
          ignition::gazebo::Joint(joint).EnablePositionCheck(ecm);
          ignition::gazebo::Joint(joint).EnableVelocityCheck(ecm);
        }
      }
      kinematics=std::make_unique<swerve::core::SwerveKinematics>(p.geometry());
      states_pub=node.Advertise<ignition::msgs::Model>("/swerve/joint_states");
      targets_pub=node.Advertise<ignition::msgs::Model>("/swerve/joint_targets");
      reference_pub=node.Advertise<ignition::msgs::Twist>("/swerve/reference_twist");
      odom_pub=node.Advertise<ignition::msgs::Odometry>("/swerve/wheel_odometry");
      configured=node.Subscribe("/kinco_swerve/cmd_vel/selected",&SwerveDrive::OnCommand,this);
      if (!configured) throw std::runtime_error("command subscription failed");
    } catch (const std::exception & e) {
      ignerr << "SwerveDrive configuration failed: " << e.what() << std::endl;
      configured=false;
    }
  }
  void PreUpdate(const ignition::gazebo::UpdateInfo & info,
    ignition::gazebo::EntityComponentManager & ecm) override {
    if (!configured) return;
    const double now=std::chrono::duration<double>(info.simTime).count();
    const double dt=std::chrono::duration<double>(info.dt).count();
    swerve::Command command;
    uint64_t generation;
    {
      std::lock_guard<std::mutex> lock(mutex);
      const bool rewind=now<last_time || info.iterations<last_iteration || dt<0;
      if (rewind || info.paused || paused) {
        mailbox.clear(); control->reset(); targets={};
        // Pausing preserves the wheel-odom pose, while a world reset starts a
        // new odometry epoch. Both discard the previous command and publisher time.
        if (rewind) odom_x=odom_y=odom_yaw=0;
        next_publish=now;
      }
      paused=info.paused;
      last_time=now; last_iteration=info.iterations;
      mailbox.consume(now,p.command_timeout);
      command=mailbox.latest; generation=mailbox.generation;
    }
    if (info.paused) return;
    swerve::Feedback feedback;
    if (!ReadFeedback(ecm,feedback)) {
      Invalidate();
      for (std::size_t i=0;i<4;++i) {
        ignition::gazebo::Joint(steering[i]).SetForce(ecm,{0});
        ignition::gazebo::Joint(wheels[i]).SetForce(ecm,{0});
      }
      return;
    }
    targets=control->step(command,generation,now,dt,feedback);
    if (!targets.feedback_valid) Invalidate();
    for (std::size_t i=0;i<4;++i) {
      const double steering_force=targets.feedback_valid ?
        p.steering_kp*(targets.steering_position[i]-feedback.steering_position[i])+
        p.steering_kd*(targets.steering_velocity[i]-feedback.steering_velocity[i]) : 0;
      // Invalid commands produce a zero wheel target, so finite feedback is
      // actively braked with the same bounded physical motor torque.
      const double wheel_force=std::isfinite(feedback.wheel_velocity[i]) ?
        p.wheel_kp*(targets.wheel_velocity[i]-feedback.wheel_velocity[i]) : 0;
      ignition::gazebo::Joint(steering[i]).SetForce(ecm,{std::clamp(steering_force,-p.steering_torque,p.steering_torque)});
      ignition::gazebo::Joint(wheels[i]).SetForce(ecm,{std::clamp(wheel_force,-p.wheel_torque,p.wheel_torque)});
    }
  }
  void PostUpdate(const ignition::gazebo::UpdateInfo & info,
    const ignition::gazebo::EntityComponentManager & ecm) override {
    if (!configured || info.paused) return;
    swerve::Feedback feedback;
    if (!ReadFeedback(ecm,feedback) || !feedback.valid(p.steering_hard_limit)) return;
    swerve::core::ModuleArray<swerve::core::ModuleState> measured;
    for (std::size_t i=0;i<4;++i) measured[i]={feedback.wheel_velocity[i]*p.wheel_radius,feedback.steering_position[i]};
    const auto estimate=kinematics->estimate_chassis_twist(measured);
    if (estimate.status!=swerve::core::SwerveStatus::kOk) return;
    const double now=std::chrono::duration<double>(info.simTime).count();
    const double dt=std::chrono::duration<double>(info.dt).count();
    const auto & twist=estimate.twist;
    if (dt>0 && dt<=.1) {
      const double mid_yaw=odom_yaw+twist.angular_z_radps*dt/2;
      odom_x+=(std::cos(mid_yaw)*twist.linear_x_mps-std::sin(mid_yaw)*twist.linear_y_mps)*dt;
      odom_y+=(std::sin(mid_yaw)*twist.linear_x_mps+std::cos(mid_yaw)*twist.linear_y_mps)*dt;
      odom_yaw=std::remainder(odom_yaw+twist.angular_z_radps*dt,2*3.14159265358979323846);
    }
    if (now+1e-9<next_publish) return;
    next_publish=now+1/p.publish_rate;
    ignition::msgs::Model states, desired;
    states.set_name("swerve"); desired.set_name("swerve_targets");
    header(states.mutable_header(),now,"base_link"); header(desired.mutable_header(),now,"base_link");
    for (std::size_t i=0;i<4;++i) {
      auto * steer=states.add_joint(); steer->set_name(prefixes[i]+"_steer_joint");
      steer->mutable_axis1()->set_position(feedback.steering_position[i]);
      steer->mutable_axis1()->set_velocity(feedback.steering_velocity[i]);
      auto * wheel=states.add_joint(); wheel->set_name(prefixes[i]+"_wheel_joint");
      wheel->mutable_axis1()->set_position(feedback.wheel_position[i]);
      wheel->mutable_axis1()->set_velocity(feedback.wheel_velocity[i]);
      auto * steer_target=desired.add_joint(); steer_target->set_name(prefixes[i]+"_steer_joint");
      steer_target->mutable_axis1()->set_position(targets.steering_position[i]);
      steer_target->mutable_axis1()->set_velocity(targets.steering_velocity[i]);
      auto * wheel_target=desired.add_joint(); wheel_target->set_name(prefixes[i]+"_wheel_joint");
      wheel_target->mutable_axis1()->set_velocity(targets.wheel_velocity[i]);
    }
    states_pub.Publish(states); targets_pub.Publish(desired);
    ignition::msgs::Twist reference;
    header(reference.mutable_header(),now,"base_link");
    reference.mutable_linear()->set_x(targets.reference.linear_x_mps);
    reference.mutable_linear()->set_y(targets.reference.linear_y_mps);
    reference.mutable_angular()->set_z(targets.reference.angular_z_radps);
    reference_pub.Publish(reference);
    ignition::msgs::Odometry odometry;
    header(odometry.mutable_header(),now,"odom");
    auto * child=odometry.mutable_header()->add_data(); child->set_key("child_frame_id"); child->add_value("base_link");
    odometry.mutable_pose()->mutable_position()->set_x(odom_x);
    odometry.mutable_pose()->mutable_position()->set_y(odom_y);
    odometry.mutable_pose()->mutable_orientation()->set_w(std::cos(odom_yaw/2));
    odometry.mutable_pose()->mutable_orientation()->set_z(std::sin(odom_yaw/2));
    odometry.mutable_twist()->mutable_linear()->set_x(twist.linear_x_mps);
    odometry.mutable_twist()->mutable_linear()->set_y(twist.linear_y_mps);
    odometry.mutable_twist()->mutable_angular()->set_z(twist.angular_z_radps);
    odom_pub.Publish(odometry);
  }
private:
  bool ReadFeedback(const ignition::gazebo::EntityComponentManager & ecm, swerve::Feedback & feedback) const {
    for (std::size_t i=0;i<4;++i) {
      const auto sp=ignition::gazebo::Joint(steering[i]).Position(ecm);
      const auto sv=ignition::gazebo::Joint(steering[i]).Velocity(ecm);
      const auto wp=ignition::gazebo::Joint(wheels[i]).Position(ecm);
      const auto wv=ignition::gazebo::Joint(wheels[i]).Velocity(ecm);
      if (!sp || sp->empty() || !sv || sv->empty() || !wp || wp->empty() || !wv || wv->empty()) return false;
      feedback.steering_position[i]=(*sp)[0]; feedback.steering_velocity[i]=(*sv)[0];
      feedback.wheel_position[i]=(*wp)[0]; feedback.wheel_velocity[i]=(*wv)[0];
    }
    return true;
  }
  void Invalidate() {
    std::lock_guard<std::mutex> lock(mutex);
    mailbox.clear(); control->reset(); targets={};
  }
  void OnCommand(const ignition::msgs::Twist & message) {
    swerve::Command command;
    command.stamp=swerve::stamp_seconds(message.header().stamp().sec(),message.header().stamp().nsec());
    for (const auto & data:message.header().data()) {
      if (data.key()=="frame_id" && data.value_size()>0) command.frame=data.value(0);
    }
    command.values={message.linear().x(),message.linear().y(),message.linear().z(),
      message.angular().x(),message.angular().y(),message.angular().z()};
    std::lock_guard<std::mutex> lock(mutex);
    if (paused) mailbox.clear();
    else mailbox.receive(command);
  }
  swerve::Parameters p;
  std::unique_ptr<swerve::Control> control;
  std::unique_ptr<swerve::core::SwerveKinematics> kinematics;
  std::array<ignition::gazebo::Entity,4> steering{},wheels{};
  ignition::transport::Node::Publisher states_pub,targets_pub,reference_pub,odom_pub;
  std::mutex mutex;
  swerve::Mailbox mailbox;
  swerve::Output targets;
  double last_time{-1},next_publish{},odom_x{},odom_y{},odom_yaw{};
  uint64_t last_iteration{};
  bool configured{},paused{};
  // Destroy transport first, while callback mutex/mailbox/parameters still live.
  ignition::transport::Node node;
};
}
IGNITION_ADD_PLUGIN(sentry_simulation::SwerveDrive,ignition::gazebo::System,
  sentry_simulation::SwerveDrive::ISystemConfigure,
  sentry_simulation::SwerveDrive::ISystemPreUpdate,
  sentry_simulation::SwerveDrive::ISystemPostUpdate)
IGNITION_ADD_PLUGIN_ALIAS(sentry_simulation::SwerveDrive,"sentry_simulation::SwerveDrive")
