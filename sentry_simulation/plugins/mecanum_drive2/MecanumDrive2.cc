// Copyright 2021 RoboMaster-OSS
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cmath>
#include <mutex>
#include <gz/common/Util.hh>
#include <gz/msgs/convert/Quaternion.hh>
#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/twist.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/Conversions.hh>

#include <gz/math/Vector3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/PID.hh>

#include "MecanumDrive2.hh"
#include "VelocityServo.hh"

#define WHEEL_NUM 4
using namespace gz;
using namespace sim;
using namespace systems;

const std::string kSdfElemJointNames[WHEEL_NUM] = {"front_right_joint", "front_left_joint", "rear_right_joint", "rear_left_joint"};

class gz::sim::systems::MecanumDrive2Private
{
public:
    void OnCmdVel(const gz::msgs::Twist &_msg);

    void UpdateOdometry(const gz::sim::UpdateInfo &_info, const gz::sim::EntityComponentManager &_ecm);

public:
    transport::Node node;
    //model
    Model model{kNullEntity};
    //chassis link
    Entity chassisLink{kNullEntity};
    //pid
    gz::math::PID xPid;
    gz::math::PID yPid;
    gz::math::PID wPid;
    //for Odometry
    std::string odomFrameId;
    std::string odomChildFrameId;
    transport::Node::Publisher odomPub;
    //velocity cmd
    msgs::Twist targetVel;
    std::mutex targetVelMutex;
};

/******************implementation for MecanumDrive2************************/
MecanumDrive2::MecanumDrive2() : dataPtr(std::make_unique<MecanumDrive2Private>())
{
}

void MecanumDrive2::Configure(const Entity &_entity,
                             const std::shared_ptr<const sdf::Element> &_sdf,
                             EntityComponentManager &_ecm,
                             EventManager & /*_eventMgr*/)
{
    this->dataPtr->model = Model(_entity);
    if (!this->dataPtr->model.Valid(_ecm))
    {
        gzerr << "MecanumDrive2 plugin should be attached to a model entity. Failed to initialize." << std::endl;
        return;
    }
    // Get params from SDF
    // Get chassis link
    const std::string chassisLinkName = _sdf->Get<std::string>("chassis_link");
    this->dataPtr->chassisLink = this->dataPtr->model.LinkByName(_ecm, chassisLinkName);
    if (this->dataPtr->chassisLink == kNullEntity)
    {
        gzerr << "chassis link with name[" << chassisLinkName << "] not found. " << std::endl;
        return;
    }
    //Get joints and links of wheel
    for (int i = 0; i < WHEEL_NUM; i++)
    {
        const std::string wheelJointName = _sdf->Get<std::string>(kSdfElemJointNames[i]);
        const Entity wheelJoint = this->dataPtr->model.JointByName(_ecm, wheelJointName);
        if (wheelJoint == kNullEntity)
        {
            gzerr << "wheel joint with name[" << wheelJointName << "] not found. " << std::endl;
            return;
        }
    }
    // Subscribe to commands
    std::string topic{this->dataPtr->model.Name(_ecm) + "/cmd_vel"};
    this->dataPtr->node.Subscribe(topic, &MecanumDrive2Private::OnCmdVel, this->dataPtr.get());
    gzmsg << "MecanumDrive2 subscribing to twist messages on [" << topic << "]" << std::endl;
    //publisher of odometry
    std::string odomTopic{this->dataPtr->model.Name(_ecm) + "/odometry"};
    this->dataPtr->odomPub = this->dataPtr->node.Advertise<msgs::Odometry>(odomTopic);
    this->dataPtr->odomFrameId=this->dataPtr->model.Name(_ecm) + "/odom" ;
    this->dataPtr->odomChildFrameId = this->dataPtr->model.Name(_ecm) + "/" + gz::common::replaceAll(chassisLinkName, "::", "/");
    // Calibrated finite-effort velocity servos. Integral effort compensates the
    // retained wheel damping/contact losses; mass and contact physics stay active.
    this->dataPtr->xPid.Init(500, 1000, 0, 150, -150, 250, -250, 0);
    this->dataPtr->yPid.Init(500, 1000, 0, 150, -150, 250, -250, 0);
    this->dataPtr->wPid.Init(200, 400, 0, 30, -30, 100, -100, 0);
}

void MecanumDrive2::PreUpdate(const gz::sim::UpdateInfo &_info,
                             gz::sim::EntityComponentManager &_ecm)
{
    if (_info.dt < std::chrono::steady_clock::duration::zero()) {
        this->dataPtr->xPid.Reset();
        this->dataPtr->yPid.Reset();
        this->dataPtr->wPid.Reset();
        std::lock_guard<std::mutex> lock(this->dataPtr->targetVelMutex);
        this->dataPtr->targetVel = msgs::Twist{};
    }
    //control for chassis
    Link chassisLink(this->dataPtr->chassisLink);
    if (!_ecm.Component<components::WorldPose>(this->dataPtr->chassisLink))
    {
        _ecm.CreateComponent(this->dataPtr->chassisLink, components::WorldPose());
    }
    if (!_ecm.Component<components::LinearVelocity>(this->dataPtr->chassisLink))
    {
        _ecm.CreateComponent(this->dataPtr->chassisLink, components::LinearVelocity());
    }
    if (!_ecm.Component<components::AngularVelocity>(this->dataPtr->chassisLink))
    {
        _ecm.CreateComponent(this->dataPtr->chassisLink, components::AngularVelocity());
    }
    // PostUpdate also reads these components, including on the first paused tick.
    if (_info.paused || _info.dt <= std::chrono::steady_clock::duration::zero()) {
        return;
    }
    //mutex for targetVel
    msgs::Twist targetVel;
    {
        std::lock_guard<std::mutex> lock(this->dataPtr->targetVelMutex);
        targetVel = this->dataPtr->targetVel;
    }
    //current state
    const auto chassisPose = _ecm.Component<components::WorldPose>(this->dataPtr->chassisLink)->Data();
    const auto linearVel = _ecm.Component<components::LinearVelocity>(this->dataPtr->chassisLink)->Data();
    const auto angularVel = _ecm.Component<components::AngularVelocity>(this->dataPtr->chassisLink)->Data();
    if (!chassisPose.IsFinite() || !linearVel.IsFinite() || !angularVel.IsFinite()) {
        this->dataPtr->xPid.Reset();
        this->dataPtr->yPid.Reset();
        this->dataPtr->wPid.Reset();
        return;
    }
    //for linear velocity control
    double xCmd = sentry_simulation::updateVelocityServo(
        this->dataPtr->xPid, targetVel.linear().x(), linearVel.X(), _info.dt);
    double yCmd = sentry_simulation::updateVelocityServo(
        this->dataPtr->yPid, targetVel.linear().y(), linearVel.Y(), _info.dt);
    //for angular velocity control
    double wCmd = sentry_simulation::updateVelocityServo(
        this->dataPtr->wPid, targetVel.angular().z(), angularVel.Z(), _info.dt);
    //force and torque on chassis link frame
    math::Vector3d tmpForce(xCmd, yCmd, 0);
    math::Vector3d tmpTorque(0, 0, wCmd);
    // transform to world frame
    auto force = chassisPose.Rot().RotateVector(tmpForce);
    auto torque = chassisPose.Rot().RotateVector(tmpTorque);
    // gzmsg << "MecanumDrive2 (force,torque):[" << force << "], [" << torque << "]" << std::endl;
    // Apply the wrench
    chassisLink.AddWorldWrench(_ecm, force, torque);
}
void MecanumDrive2::PostUpdate(const gz::sim::UpdateInfo &_info,
                              const gz::sim::EntityComponentManager &_ecm)
{

    // 1.check collsion  of wheel's link and set the wheel's state true if the wheel contacts with ground plane, and
    // then can compute force and torque based wheel states. (TODO)
    // 2.for odometer
    this->dataPtr->UpdateOdometry(_info, _ecm);
}

/******************implementation for MecanumDrive2Private******************/

void MecanumDrive2Private::OnCmdVel(const gz::msgs::Twist &_msg)
{
    std::lock_guard<std::mutex> lock(this->targetVelMutex);
    this->targetVel = std::isfinite(_msg.linear().x()) &&
        std::isfinite(_msg.linear().y()) && std::isfinite(_msg.angular().z()) ?
        _msg : msgs::Twist{};
    //gzmsg << "MecanumDrive2 msg x: [" << _msg.linear().x() << "]" << std::endl;
}

void MecanumDrive2Private::UpdateOdometry(const gz::sim::UpdateInfo &_info,
                                         const gz::sim::EntityComponentManager &_ecm)
{
    //get pose and velocity of chassis
    const auto chassisPose = _ecm.Component<components::WorldPose>(this->chassisLink)->Data();
    const auto linearVel = _ecm.Component<components::LinearVelocity>(this->chassisLink)->Data();
    const auto angularVel = _ecm.Component<components::AngularVelocity>(this->chassisLink)->Data();
    // Construct the odometry message and publish it.
    msgs::Odometry msg;
    msg.mutable_pose()->mutable_position()->set_x(chassisPose.X());
    msg.mutable_pose()->mutable_position()->set_y(chassisPose.Y());
    math::Quaterniond orientation(0, 0, chassisPose.Yaw());
    msgs::Set(msg.mutable_pose()->mutable_orientation(), orientation);

    msg.mutable_twist()->mutable_linear()->set_x(linearVel.X());
    msg.mutable_twist()->mutable_linear()->set_y(linearVel.Y());
    msg.mutable_twist()->mutable_angular()->set_z(angularVel.Z());
    // Set the time stamp in the header
    msg.mutable_header()->mutable_stamp()->CopyFrom(convert<msgs::Time>(_info.simTime));
    // Set the frame id.
    auto frame = msg.mutable_header()->add_data();
    frame->set_key("frame_id");
    frame->add_value(this->odomFrameId);
    auto childFrame = msg.mutable_header()->add_data();
    childFrame->set_key("child_frame_id");
    childFrame->add_value(this->odomChildFrameId);
    // Publish the message
    this->odomPub.Publish(msg);
}

/******************register*************************************************/
GZ_ADD_PLUGIN(MecanumDrive2,
                    gz::sim::System,
                    MecanumDrive2::ISystemConfigure,
                    MecanumDrive2::ISystemPreUpdate,
                    MecanumDrive2::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(MecanumDrive2, "gz::sim::systems::MecanumDrive2")
