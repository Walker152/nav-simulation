#include <gtest/gtest.h>
#include <chrono>
#include <cmath>
#include <limits>
#include "SwerveControl.hpp"
using namespace sentry_simulation::swerve;

TEST(CommandContract, RejectsMalformedAndExpiredIncludingUnusedComponents) {
  Command c; c.stamp = 1; c.frame = "base_link";
  EXPECT_TRUE(c.valid(1.1, .25));
  for (int i = 0; i < 6; ++i) {
    auto bad = c; bad.values[i] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(bad.valid(1.1, .25));
  }
  EXPECT_FALSE(c.valid(.99, .25));
  EXPECT_FALSE(c.valid(1.251, .25));
  c.stamp = 0; EXPECT_FALSE(c.valid(0, .25));
  c.stamp = 1; c.frame = "odom"; EXPECT_FALSE(c.valid(1, .25));
}
TEST(CommandContract, InvalidThenFreshZeroPreservesRevocation) {
  Mailbox mailbox;
  Command c; c.stamp = 1; c.values[0] = 1;
  mailbox.receive(c);
  const auto generation = mailbox.generation;
  c.stamp = 0; mailbox.receive(c);
  c.stamp = 1; c.values[0] = 0; mailbox.receive(c);
  EXPECT_GT(mailbox.generation, generation);
  EXPECT_TRUE(mailbox.latest.valid(1, .25));
  mailbox.clear(); EXPECT_FALSE(mailbox.latest.valid(1, .25));
}
TEST(Reference, VectorAccelerationAndYawAreBounded) {
  Control controller{Parameters{}}; Feedback f{}; Command c;
  c.stamp = 1; c.values = {10, 10, 0, 0, 0, 10};
  auto output = controller.step(c, 0, 1, .002, f);
  EXPECT_NEAR(std::hypot(output.reference.linear_x_mps, output.reference.linear_y_mps), .002, 1e-12);
  EXPECT_NEAR(output.reference.angular_z_radps, .004, 1e-12);
  for (int i=0; i<2000; ++i) output = controller.step(c,0,1,.002,f);
  EXPECT_NEAR(std::hypot(output.reference.linear_x_mps, output.reference.linear_y_mps), 2, 1e-10);
  EXPECT_NEAR(output.reference.angular_z_radps, 2, 1e-10);
}
TEST(Reference, FreshZeroDeceleratesButInvalidImmediatelyStopsAndLatches) {
  Control controller{Parameters{}}; Feedback f{}; Command c; c.stamp = 1; c.values[0] = 1;
  for (int i=0;i<100;++i) controller.step(c,0,1,.002,f);
  c.values[0]=0;
  auto o=controller.step(c,0,1,.002,f); EXPECT_GT(o.reference.linear_x_mps,.19);
  c.stamp=0; f.steering_position.fill(.4);
  o=controller.step(c,1,1,.002,f);
  EXPECT_EQ(o.reference.linear_x_mps,0); EXPECT_EQ(o.wheel_velocity[0],0);
  EXPECT_EQ(o.steering_position[0],.4);
  f.steering_position.fill(.5); o=controller.step(c,1,1,.002,f);
  EXPECT_EQ(o.steering_position[0],.4);
  c.stamp=1; o=controller.step(c,1,1,.002,f);
  EXPECT_EQ(o.reference.linear_x_mps,0); EXPECT_EQ(o.steering_position[0],.5);
}
TEST(Reference, RevocationCannotHideBehindLatestValidZero) {
  Control controller{Parameters{}}; Feedback f{}; Command c; c.stamp=1; c.values[0]=1;
  for(int i=0;i<100;++i) controller.step(c,0,1,.002,f);
  c.values[0]=0;
  auto o=controller.step(c,1,1,.002,f);
  EXPECT_EQ(o.reference.linear_x_mps,0);
  controller.reset(); c.values[0]=1;
  o=controller.step(c,1,1,.002,f); EXPECT_NEAR(o.reference.linear_x_mps,.002,1e-12);
}
TEST(SteeringProfile, ReversalAndEndpointRespectVelocityAcceleration) {
  SteeringProfile profile;
  double previous_velocity=0;
  for(int i=0;i<8000;++i) {
    const double target=i<800 ? 2.7052603406 : (i<3000 ? -2.7052603406 : 2.7052603406);
    profile.step(target,.002,6.28318530718,12.5663706144);
    EXPECT_LE(std::abs(profile.velocity),6.28318530718+1e-10);
    EXPECT_LE(std::abs(profile.velocity-previous_velocity),12.5663706144*.002+1e-10);
    EXPECT_LE(std::abs(profile.position),2.7052603406+1e-9);
    previous_velocity=profile.velocity;
  }
  EXPECT_NEAR(profile.position,2.7052603406,1e-8);
}
TEST(Reference, BadFeedbackAndBadDtStopWithoutNonFiniteOutput) {
  Control controller{Parameters{}}; Feedback f{}; Command c; c.stamp=1; c.values[0]=1;
  f.steering_position[0]=std::numeric_limits<double>::quiet_NaN();
  auto o=controller.step(c,0,1,.002,f); EXPECT_FALSE(o.feedback_valid); EXPECT_EQ(o.wheel_velocity[0],0);
  f.steering_position[0]=3;
  o=controller.step(c,0,1,.002,f); EXPECT_FALSE(o.feedback_valid);
  f.steering_position[0]=0;
  o=controller.step(c,0,1,-.002,f); EXPECT_EQ(o.wheel_velocity[0],0);
}
TEST(Reference, LateralMotionWaitsForMeasuredAlignment) {
  Control controller{Parameters{}}; Feedback f{}; Command c; c.stamp=1; c.values[1]=1;
  auto o=controller.step(c,0,1,.002,f);
  EXPECT_EQ(o.wheel_velocity[0],0);
  for(int i=0;i<1000;++i) {f.steering_position=o.steering_position; o=controller.step(c,0,1,.002,f);}
  EXPECT_GT(o.wheel_velocity[0],14); EXPECT_NEAR(o.steering_position[0],1.57079632679,1e-7);
}
TEST(Core, CommonWheelSaturationAndActualOdometry) {
  Parameters p; auto geometry=p.geometry();
  kinco_swerve_core::SwerveKinematics k(geometry);
  kinco_swerve_core::ModuleCommandOptimizer optimizer(geometry,{3,1e-5});
  auto velocities=k.to_module_velocities({3,1,2});
  kinco_swerve_core::ModuleArray<kinco_swerve_core::SteeringReference> steering{};
  auto output=optimizer.optimize(velocities.velocities,steering);
  EXPECT_LT(output.saturation_scale,1);
  kinco_swerve_core::ModuleArray<kinco_swerve_core::ModuleState> actual{};
  for(std::size_t i=0;i<4;++i) {
    EXPECT_LE(std::abs(output.setpoints[i].wheel_speed_mps),3);
    actual[i]={output.setpoints[i].wheel_speed_mps,output.setpoints[i].steering_angle_rad};
  }
  const auto estimate=k.estimate_chassis_twist(actual);
  EXPECT_NEAR(estimate.twist.linear_x_mps,3*output.saturation_scale,1e-10);
  EXPECT_NEAR(estimate.twist.linear_y_mps,output.saturation_scale,1e-10);
  EXPECT_NEAR(estimate.twist.angular_z_radps,2*output.saturation_scale,1e-10);
}
TEST(CommandContract, FutureRejectedMessageCannotBecomeFreshWithoutNewPacket) {
  Mailbox mailbox; Command c; c.stamp=2; c.values[0]=1;
  mailbox.receive(c);
  mailbox.consume(1.002,.25);
  EXPECT_FALSE(mailbox.latest.valid(2,.25));
}
TEST(Reference, InvalidParametersFailExplicitly) {
  Parameters p; p.linear_acceleration=0; EXPECT_THROW(Control{p},std::invalid_argument);
  p=Parameters{}; p.steering_limit=p.steering_hard_limit; EXPECT_THROW(Control{p},std::invalid_argument);
}
TEST(SteeringProfile, MovingTargetsRemainInsideEndpoints) {
  SteeringProfile profile; double previous_velocity=0;
  for(int i=0;i<10000;++i) {
    const double target=2.7052603406*std::sin(i*.023);
    profile.step(target,.002,6.28318530718,12.5663706144);
    EXPECT_LE(std::abs(profile.position),2.7052603406+1e-9);
    EXPECT_LE(std::abs(profile.velocity-previous_velocity),.02513274123);
    previous_velocity=profile.velocity;
  }
}
TEST(Reference, RepeatedInvalidPacketsDoNotRelatchDisturbedSteering) {
  Control controller{Parameters{}}; Feedback f{}; Command c; c.stamp=1; c.values[0]=1;
  controller.step(c,0,1,.002,f);
  c.stamp=0; f.steering_position.fill(.4);
  controller.step(c,1,1,.002,f);
  f.steering_position.fill(.6);
  const auto output=controller.step(c,2,1,.002,f);
  EXPECT_EQ(output.steering_position[0],.4);
}
TEST(CommandContract, SameNanosecondStampDoesNotBecomeFutureAfterConversion) {
  // Simulator duration conversion divides once; protobuf conversion combines
  // sec plus nsec*1e-9. These can differ by one floating-point ULP.
  Command c;
  for (int64_t ns=2000000;ns<10000000000LL;ns+=2000000) {
    const double now=std::chrono::duration<double>(std::chrono::nanoseconds(ns)).count();
    c.stamp=static_cast<double>(ns/1000000000)+(ns%1000000000)*1e-9;
    ASSERT_TRUE(c.valid(now,.25)) << ns;
    c.stamp=static_cast<double>((ns+1)/1000000000)+((ns+1)%1000000000)*1e-9;
    ASSERT_FALSE(c.valid(now,.25)) << ns;
  }
}

TEST(CommandContract, PublishedClockAheadOfPreviousUpdateAcceptedAtFirstConsumer) {
  Mailbox mailbox; Command c; c.stamp=47.388; c.values[0]=1;
  mailbox.receive(c);
  mailbox.consume(47.388,.25);
  EXPECT_TRUE(mailbox.latest.valid(47.388,.25));
  EXPECT_EQ(mailbox.generation,0);
}
TEST(CommandContract, HiddenFutureAndStalePacketsRevokeAtFirstConsumer) {
  for (double invalid_stamp : {2.0,.75}) {
    Mailbox mailbox; Command c; c.stamp=invalid_stamp; c.values[0]=1;
    mailbox.receive(c);
    c.stamp=1.002; c.values[0]=0;
    mailbox.receive(c);
    mailbox.consume(1.002,.25);
    EXPECT_GT(mailbox.generation,0);
    EXPECT_TRUE(mailbox.latest.valid(1.002,.25));
    const auto generation=mailbox.generation;
    mailbox.consume(1.004,.25);
    EXPECT_EQ(mailbox.generation,generation);
  }
}
TEST(CommandContract, ClearDropsPendingEpochAndNoPacketExpiresNormally) {
  Mailbox mailbox; Command c; c.stamp=10;
  mailbox.receive(c);
  mailbox.clear();
  const auto generation=mailbox.generation;
  c.stamp=1; mailbox.receive(c); mailbox.consume(1,.25);
  EXPECT_EQ(mailbox.generation,generation);
  EXPECT_TRUE(mailbox.latest.valid(1,.25));
  mailbox.consume(1.251,.25);
  EXPECT_FALSE(mailbox.latest.valid(1.251,.25));
}
TEST(CommandContract, RejectsMalformedProtobufNanosecondsWithoutNormalization) {
  Command c;
  c.stamp=stamp_seconds(1,1000000000); EXPECT_FALSE(c.valid(2,.25));
  c.stamp=stamp_seconds(1,-1); EXPECT_FALSE(c.valid(1,.25));
  c.stamp=stamp_seconds(-1,0); EXPECT_FALSE(c.valid(1,.25));
  c.stamp=stamp_seconds(1,999999999); EXPECT_TRUE(c.valid(1.999999999,.25));
}
