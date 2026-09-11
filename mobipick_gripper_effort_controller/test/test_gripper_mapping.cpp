#include <gtest/gtest.h>

#include <cmath>

#include <mobipick_gripper_effort_controller/gripper_mapping.h>

using mobipick_gripper_effort_controller::clampTorque;
using mobipick_gripper_effort_controller::EffortMapping;
using mobipick_gripper_effort_controller::GapJointMapping;
using mobipick_gripper_effort_controller::stepTowards;

TEST(GapJointMapping, endpointsAndMidpoint)
{
  GapJointMapping m;  // defaults: gap 0.14 -> 0.0 rad, gap 0.0 -> 0.755 rad
  EXPECT_TRUE(m.valid());
  EXPECT_DOUBLE_EQ(0.0, m.gapToJoint(0.14));
  EXPECT_DOUBLE_EQ(0.755, m.gapToJoint(0.0));
  EXPECT_NEAR(0.3775, m.gapToJoint(0.07), 1e-9);
  EXPECT_NEAR(0.07, m.jointToGap(0.3775), 1e-9);
}

TEST(GapJointMapping, matchesOldBridgeFormula)
{
  // The removed robotiq_2f_140_command_bridge used angle = 0.76 - 0.76 / 0.14 * gap
  GapJointMapping m;
  m.joint_closed = 0.76;
  for (double gap = 0.0; gap <= 0.14; gap += 0.01)
  {
    EXPECT_NEAR(-0.76 / 0.14 * gap + 0.76, m.gapToJoint(gap), 1e-9);
  }
}

TEST(GapJointMapping, clampsOutOfRangeGap)
{
  GapJointMapping m;
  EXPECT_DOUBLE_EQ(0.755, m.gapToJoint(-0.02));
  EXPECT_DOUBLE_EQ(0.0, m.gapToJoint(0.5));
  EXPECT_DOUBLE_EQ(0.755, m.clampJoint(2.0));
  EXPECT_DOUBLE_EQ(0.0, m.clampJoint(-1.0));
}

TEST(GapJointMapping, toleranceConversion)
{
  GapJointMapping m;
  EXPECT_NEAR(0.005 * 0.755 / 0.14, m.gapToleranceToJoint(0.005), 1e-12);
  EXPECT_GT(m.gapToleranceToJoint(-0.005), 0.0);
}

TEST(GapJointMapping, acceptsSameGoalRangeAsRealServer)
{
  // mobipick_bringup_control.launch: min_gap -0.015, max_gap 0.140
  GapJointMapping m;
  EXPECT_TRUE(m.accepted(0.0));
  EXPECT_TRUE(m.accepted(0.14));
  EXPECT_TRUE(m.accepted(-0.015));
  EXPECT_TRUE(m.accepted(-0.01));
  EXPECT_FALSE(m.accepted(-0.02));
  EXPECT_FALSE(m.accepted(0.141));
  EXPECT_FALSE(m.accepted(0.755));  // MoveIt sending the joint angle as gap, see mobipick README
  EXPECT_FALSE(m.accepted(std::nan("")));
  // accepted but below gap_closed: executed as fully closed (Robotiq register saturates)
  EXPECT_DOUBLE_EQ(0.755, m.gapToJoint(-0.01));
}

TEST(GapJointMapping, rejectsDegenerateRanges)
{
  GapJointMapping m;
  m.gap_open = m.gap_closed;
  EXPECT_FALSE(m.valid());
  m = GapJointMapping();
  m.joint_open = m.joint_closed;
  EXPECT_FALSE(m.valid());
  m = GapJointMapping();
  m.accept_min_gap = m.accept_max_gap;
  EXPECT_FALSE(m.valid());
}

TEST(EffortMapping, linearInsideRange)
{
  EffortMapping e;  // 30..100 -> 0.1..0.6 Nm
  EXPECT_TRUE(e.valid());
  EXPECT_DOUBLE_EQ(0.1, e.toTorqueLimit(30.0));
  EXPECT_DOUBLE_EQ(0.6, e.toTorqueLimit(100.0));
  EXPECT_NEAR(0.1 + 20.0 / 70.0 * 0.5, e.toTorqueLimit(50.0), 1e-12);
  EXPECT_TRUE(e.accepted(50.0));
}

TEST(EffortMapping, acceptsSameEffortRangeAsRealServer)
{
  // robotiq_2f_gripper_action_server.launch: min_effort 30, max_effort 100; anything else is refused,
  // including 0 ("do not limit" in control_msgs).
  EffortMapping e;
  EXPECT_TRUE(e.accepted(30.0));
  EXPECT_TRUE(e.accepted(100.0));
  EXPECT_FALSE(e.accepted(29.9));
  EXPECT_FALSE(e.accepted(100.1));
  EXPECT_FALSE(e.accepted(0.0));
  EXPECT_FALSE(e.accepted(-5.0));
  EXPECT_FALSE(e.accepted(std::nan("")));
}

TEST(EffortMapping, torqueLimitIsClampedToConfiguredRange)
{
  EffortMapping e;
  EXPECT_DOUBLE_EQ(0.1, e.toTorqueLimit(1.0));
  EXPECT_DOUBLE_EQ(0.6, e.toTorqueLimit(150.0));
  EXPECT_DOUBLE_EQ(0.6, e.toTorqueLimit(e.hold_input));
  e.hold_input = 30.0;
  EXPECT_DOUBLE_EQ(0.1, e.toTorqueLimit(e.hold_input));
}

TEST(EffortMapping, rejectsInvalid)
{
  EffortMapping e;
  e.input_max = e.input_min;
  EXPECT_FALSE(e.valid());
  e = EffortMapping();
  e.torque_max = 0.05;  // below torque_min
  EXPECT_FALSE(e.valid());
}

TEST(StepTowards, rampsAndSnaps)
{
  EXPECT_DOUBLE_EQ(0.1, stepTowards(0.0, 1.0, 0.1));
  EXPECT_DOUBLE_EQ(-0.1, stepTowards(0.0, -1.0, 0.1));
  EXPECT_DOUBLE_EQ(1.0, stepTowards(0.95, 1.0, 0.1));
  EXPECT_DOUBLE_EQ(1.0, stepTowards(1.0, 1.0, 0.1));
  EXPECT_DOUBLE_EQ(0.1, stepTowards(0.0, 1.0, -0.1));
  // 0.5 rad/s at a 1 ms step reaches 0.755 rad in ~1510 steps (+-1 for float accumulation)
  double x = 0.0;
  int n = 0;
  while (x != 0.755 && n < 5000) { x = stepTowards(x, 0.755, 0.5 * 0.001); ++n; }
  EXPECT_DOUBLE_EQ(0.755, x);
  EXPECT_TRUE(n >= 1509 && n <= 1511);
}

TEST(ClampTorque, symmetricSaturation)
{
  EXPECT_DOUBLE_EQ(2.0, clampTorque(10.0, 2.0));
  EXPECT_DOUBLE_EQ(-2.0, clampTorque(-10.0, 2.0));
  EXPECT_DOUBLE_EQ(0.5, clampTorque(0.5, 2.0));
  EXPECT_DOUBLE_EQ(2.0, clampTorque(10.0, -2.0));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
