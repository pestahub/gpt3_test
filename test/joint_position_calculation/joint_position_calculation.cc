#include <gtest/gtest.h>
#include <joint_imitator.h>

TEST(SetParams, JointImitator) {
  JointImitator joint_imitator;

  joint_imitator.setMode(JointMode::VELOCITY);
  EXPECT_EQ(joint_imitator.getMode(), JointMode::VELOCITY);

  joint_imitator.setRate(100);
  EXPECT_EQ(joint_imitator.getRate(), 100);

  joint_imitator.setProfileVelocity(0.5);
  EXPECT_EQ(joint_imitator.getProfileVelocity(), 0.5);

  joint_imitator.setProfileAcceleration(0.7);
  EXPECT_EQ(joint_imitator.getProfileAcceleration(), 0.7);
}

void checkSetVelocity(JointImitator& joint_imitator, double setpoint_velocity,
                      double profile_acceleration) {
  double start_velocity = joint_imitator.getVelocity();
  joint_imitator.setVelocity(setpoint_velocity);
  joint_imitator.setProfileAcceleration(profile_acceleration);
  double timeout =
      std::fabs(setpoint_velocity - start_velocity) / profile_acceleration + 1;

  while (joint_imitator.getVelocity() != setpoint_velocity) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));

    timeout -= 0.002;
    if (timeout < 0) {
      FAIL() << "Timeout while waiting for velocity to reach setpoint 2, "
                "velocity: "
             << joint_imitator.getVelocity();
    }
  }
}

TEST(VelocityImitation, JointImitator) {
  JointImitator joint_imitator;

  joint_imitator.setMode(JointMode::VELOCITY);
  joint_imitator.setRate(100);
  joint_imitator.setProfileVelocity(0.5);
  joint_imitator.setProfileAcceleration(0.7);

  checkSetVelocity(joint_imitator, 1, 0.7);

  joint_imitator.setProfileAcceleration(1000);
  checkSetVelocity(joint_imitator, -1, 1000);

  joint_imitator.setProfileAcceleration(1);
  checkSetVelocity(joint_imitator, 0, 1);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}