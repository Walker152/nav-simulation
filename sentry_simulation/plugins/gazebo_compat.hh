#pragma once

// The algorithms are shared; only the Gazebo API naming changed between releases.
#ifdef SENTRY_GAZEBO_FORTRESS
#include <ignition/common/Console.hh>
#include <ignition/common/Util.hh>
#include <ignition/math/PID.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/msgs/Utility.hh>
#include <ignition/msgs/model.pb.h>
#include <ignition/msgs/odometry.pb.h>
#include <ignition/msgs/twist.pb.h>
#include <ignition/plugin/Register.hh>
#include <ignition/gazebo/Conversions.hh>
#include <ignition/gazebo/Joint.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/AngularVelocity.hh>
#include <ignition/gazebo/components/LinearVelocity.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/transport/Node.hh>
#define SENTRY_GZ ignition
#define SENTRY_SIM gazebo
#define GZ_SIM_VISIBLE IGNITION_GAZEBO_VISIBLE
#define GZ_ADD_PLUGIN IGNITION_ADD_PLUGIN
#define GZ_ADD_PLUGIN_ALIAS IGNITION_ADD_PLUGIN_ALIAS
#define gzerr ignerr
#define gzmsg ignmsg
#else
#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/math/PID.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/convert/Quaternion.hh>
#include <gz/msgs/model.pb.h>
#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/twist.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Conversions.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/transport/Node.hh>
#define SENTRY_GZ gz
#define SENTRY_SIM sim
#endif
