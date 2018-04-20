#include <chrono>
#include <math.h>
#include <thread>

#include "ros/ros.h"
#include "gtest/gtest.h"

#include "ArgoGlobals.hpp"
#include "Subscriber.hpp"
#include "argo_driver/Wheels.h"

namespace {
const std::string HANDLE_PATH{"argo_driver_test"};
const std::string SET_SPEED_SERV_NAME{"cmd_wheel_speeds"};
const std::string SET_TWIST_NAME{"cmd_vel"};

const int MAX_TOPIC_LEN = 1;

int calculateChangeInVelocity(double angularMomentum) {
  double velocityChange = (angularMomentum * g_LENGTH_BETWEEN_WHEELS) / 2;
  return velocityChange * METERS_TO_MILLIS;
}

void rosSpinAndSleep() {
  using namespace std::chrono_literals;
  for (int i = 0; i < 5; i++) {
    std::this_thread::sleep_for(50ms);
    ros::spinOnce();
  }
}

} // namespace

TEST(Subscriber_Speed, targetSpeedSetsCorrectly) {
  ros::NodeHandle handle(HANDLE_PATH);
  Subscriber testInstance(handle);

  ros::Publisher publisherHandle =
      handle.advertise<argo_driver::Wheels>(SET_SPEED_SERV_NAME, MAX_TOPIC_LEN);

  const double expectedLeftSpeed = 12.34;
  const double expectedRightSpeed = 23.45;

  argo_driver::Wheels msg;
  msg.leftWheel = expectedLeftSpeed;
  msg.rightWheel = expectedRightSpeed;

  publisherHandle.publish(std::move(msg));

  rosSpinAndSleep();

  auto targetSpeed = testInstance.getLastSpeedTarget();
  EXPECT_EQ(targetSpeed.leftWheel, expectedLeftSpeed * METERS_TO_MILLIS);
  EXPECT_EQ(targetSpeed.rightWheel, expectedRightSpeed * METERS_TO_MILLIS);
}

TEST(Subscriber_Twist, tarrgetSpeedSetsCorrectly) {
  ros::NodeHandle handle(HANDLE_PATH);
  Subscriber testInstance(handle);

  ros::Publisher publisherHandle =
      handle.advertise<geometry_msgs::Twist>(SET_TWIST_NAME, MAX_TOPIC_LEN);

  const double changeInRad = M_PI_4; // Rad / s
  const double velocity = 10;        // m/s

  // Convention is "anti clockwise" around Z axis so need to negate
  const int expectedVelocityDiff = -calculateChangeInVelocity(changeInRad);

  geometry_msgs::Twist msg;
  msg.linear.x = velocity;
  msg.angular.z = changeInRad;

  publisherHandle.publish(std::move(msg));

  rosSpinAndSleep();

  const double expectedLeftSpeed =
      (velocity * METERS_TO_MILLIS) + expectedVelocityDiff;
  const double expectedRightSpeed =
      (velocity * METERS_TO_MILLIS) - expectedVelocityDiff;

  auto targetSpeed = testInstance.getLastSpeedTarget();
  EXPECT_EQ(targetSpeed.leftWheel, expectedLeftSpeed);
  EXPECT_EQ(targetSpeed.rightWheel, expectedRightSpeed);
}
