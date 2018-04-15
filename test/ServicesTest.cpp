#include <atomic>
#include <chrono>
#include <functional>
#include <math.h>
#include <thread>

#include "ros/ros.h"
#include "gtest/gtest.h"

#include "ArgoGlobals.hpp"
#include "Services.hpp"

namespace {
const std::string HANDLE_PATH{"argo_driver_test"};
const std::string SET_SPEED_SERV_NAME{"set_target_speed"};
const std::string SET_ODOM_SERV_NAME{"set_target_odom"};
const std::string STOP_SERV_NAME{"stop_vehicle"};

const double TURN_SPEED = 1; // Turn speed per wheel as defined in Services.cpp

void rosSpinThread(std::atomic<bool> &runThread) {
  while (runThread) {
    ros::spinOnce();
  }
}

int calculateChangeInVelocity(double radians, double time) {
  double angularMomentum = radians / time;
  double velocityChange = (angularMomentum * g_LENGTH_BETWEEN_WHEELS) / 2;
  return velocityChange * METERS_TO_MILLIS;
}

int calculateChangeInVelocity(double radians, double distance,
                              double velocity) {
  double deltaTime = distance / velocity;
  return calculateChangeInVelocity(radians, deltaTime);
}

double convertToRad(double degrees) { return degrees * (M_PI / 180); }

} // namespace

TEST(Services_Speed, targetSpeedSetsCorrectly) {
  ros::NodeHandle handle(HANDLE_PATH);
  Services testInstance(handle);

  ros::ServiceClient speedClient =
      handle.serviceClient<argo_driver::SetWheelSpeeds>(
          '/' + HANDLE_PATH + '/' + SET_SPEED_SERV_NAME);

  const double expectedLeftSpeed = 12.34;
  const double expectedRightSpeed = 23.45;

  argo_driver::SetWheelSpeeds msg;
  msg.request.leftWheelTarget = expectedLeftSpeed;
  msg.request.rightWheelTarget = expectedRightSpeed;

  std::atomic<bool> runThread{true};
  std::thread rosLoopRunner = std::thread(rosSpinThread, std::ref(runThread));
  ASSERT_TRUE(speedClient.call(msg));

  // Stop thread calling ros::spin
  runThread = false;
  rosLoopRunner.join();

  auto targetSpeed = testInstance.getTargetSpeed();
  EXPECT_EQ(targetSpeed.leftWheel, expectedLeftSpeed * METERS_TO_MILLIS);
  EXPECT_EQ(targetSpeed.rightWheel, expectedRightSpeed * METERS_TO_MILLIS);
}

TEST(Services_Odom, targetOdomDegrees) {
  ros::NodeHandle handle(HANDLE_PATH);
  Services testInstance(handle);

  ros::ServiceClient odomClient =
      handle.serviceClient<argo_driver::SetTargetOdom>('/' + HANDLE_PATH + '/' +
                                                       SET_ODOM_SERV_NAME);

  const double distance = 1;         // meter
  const double changeInDegrees = 20; // Need to turn 20 degrees in 1 meter
  const double changeInRad =
      convertToRad(changeInDegrees); // Used for our calcs
  const double velocity = 10;        // m/s
  const int expectedVelocityDiff =
      calculateChangeInVelocity(changeInRad, distance, velocity);

  argo_driver::SetTargetOdom msg;
  msg.request.isRadians = false;
  msg.request.targetClockwiseDegreesRotation = changeInDegrees;
  msg.request.targetClockwiseRadiansRotation = 0;
  msg.request.targetDistance = distance;
  msg.request.targetSpeed = velocity;

  std::atomic<bool> runThread{true};
  std::thread rosLoopRunner = std::thread(rosSpinThread, std::ref(runThread));
  ASSERT_TRUE(odomClient.call(msg));

  // Stop thread calling ros::spin
  runThread = false;
  rosLoopRunner.join();

  const double expectedLeftSpeed =
      (velocity * METERS_TO_MILLIS) + expectedVelocityDiff;
  const double expectedRightSpeed =
      (velocity * METERS_TO_MILLIS) - expectedVelocityDiff;

  auto targetSpeed = testInstance.getTargetSpeed();
  EXPECT_EQ(targetSpeed.leftWheel, expectedLeftSpeed);
  EXPECT_EQ(targetSpeed.rightWheel, expectedRightSpeed);
}

TEST(Services_Odom, targetOdomRadians) {
  ros::NodeHandle handle(HANDLE_PATH);
  Services testInstance(handle);

  ros::ServiceClient odomClient =
      handle.serviceClient<argo_driver::SetTargetOdom>('/' + HANDLE_PATH + '/' +
                                                       SET_ODOM_SERV_NAME);

  const double distance = 1;         // meter
  const double changeInRad = M_PI_4; // or 45 degrees
  const double velocity = 10;        // m/s
  const int expectedVelocityDiff =
      calculateChangeInVelocity(changeInRad, distance, velocity);

  argo_driver::SetTargetOdom msg;
  msg.request.isRadians = true;
  msg.request.targetClockwiseDegreesRotation = 0;
  msg.request.targetClockwiseRadiansRotation = changeInRad;
  msg.request.targetDistance = distance;
  msg.request.targetSpeed = velocity;

  std::atomic<bool> runThread{true};
  std::thread rosLoopRunner = std::thread(rosSpinThread, std::ref(runThread));
  ASSERT_TRUE(odomClient.call(msg));

  // Stop thread calling ros::spin
  runThread = false;
  rosLoopRunner.join();

  const double expectedLeftSpeed =
      (velocity * METERS_TO_MILLIS) + expectedVelocityDiff;
  const double expectedRightSpeed =
      (velocity * METERS_TO_MILLIS) - expectedVelocityDiff;

  auto targetSpeed = testInstance.getTargetSpeed();
  EXPECT_EQ(targetSpeed.leftWheel, expectedLeftSpeed);
  EXPECT_EQ(targetSpeed.rightWheel, expectedRightSpeed);
}

TEST(Services_Odom, targetOdomTurnsOnSpot) {
  ros::NodeHandle handle(HANDLE_PATH);
  Services testInstance(handle);

  ros::ServiceClient odomClient =
      handle.serviceClient<argo_driver::SetTargetOdom>('/' + HANDLE_PATH + '/' +
                                                       SET_ODOM_SERV_NAME);

  const double distance = 0;         // meter
  const double changeInRad = M_PI_4; // or 45 degrees
  const double velocity = 10;        // m/s

  argo_driver::SetTargetOdom msg;
  msg.request.isRadians = true;
  msg.request.targetClockwiseDegreesRotation = 0;
  msg.request.targetClockwiseRadiansRotation = changeInRad;
  msg.request.targetDistance = distance;
  msg.request.targetSpeed = velocity;

  std::atomic<bool> runThread{true};
  std::thread rosLoopRunner = std::thread(rosSpinThread, std::ref(runThread));

  ASSERT_TRUE(odomClient.call(msg));

  runThread = false;
  rosLoopRunner.join();

  auto targetSpeed = testInstance.getTargetSpeed();
  EXPECT_EQ(targetSpeed.leftWheel, TURN_SPEED * METERS_TO_MILLIS);
  EXPECT_EQ(targetSpeed.rightWheel, -TURN_SPEED * METERS_TO_MILLIS);
}

TEST(Services_Odom, turnStopsAfterTime) {
  ros::NodeHandle handle(HANDLE_PATH);
  Services testInstance(handle);

  ros::ServiceClient odomClient =
      handle.serviceClient<argo_driver::SetTargetOdom>('/' + HANDLE_PATH + '/' +
                                                       SET_ODOM_SERV_NAME);

  const double distance = 0;         // meter
  const double changeInRad = M_PI_4; // or 45 degrees
  const double velocity = 10;        // m/s

  const double timeForTurn =
      changeInRad / (TURN_SPEED / g_LENGTH_BETWEEN_WHEELS);

  argo_driver::SetTargetOdom msg;
  msg.request.isRadians = true;
  msg.request.targetClockwiseDegreesRotation = 0;
  msg.request.targetClockwiseRadiansRotation = changeInRad;
  msg.request.targetDistance = distance;
  msg.request.targetSpeed = velocity;

  std::atomic<bool> runThread{true};
  std::thread rosLoopRunner = std::thread(rosSpinThread, std::ref(runThread));

  ASSERT_TRUE(odomClient.call(msg));

  // After n seconds we should stop turning
  testInstance.startTimers();
  std::chrono::duration<double> delayTime(timeForTurn * 1.2);
  std::this_thread::sleep_for(delayTime);

  // Stop thread calling ros::spin
  runThread = false;
  rosLoopRunner.join();

  auto targetSpeed = testInstance.getTargetSpeed();
  EXPECT_EQ(targetSpeed.leftWheel, 0);
  EXPECT_EQ(targetSpeed.rightWheel, 0);
}

TEST(Services_Odom, turnOnSpotStopsAfterTime) {
  ros::NodeHandle handle(HANDLE_PATH);
  Services testInstance(handle);

  ros::ServiceClient odomClient =
      handle.serviceClient<argo_driver::SetTargetOdom>('/' + HANDLE_PATH + '/' +
                                                       SET_ODOM_SERV_NAME);

  const double distance = 10;        // meter
  const double changeInRad = M_PI_4; // or 45 degrees
  const double velocity = 10;        // m/s

  const double timeForTurn =
      changeInRad / (TURN_SPEED / g_LENGTH_BETWEEN_WHEELS);

  argo_driver::SetTargetOdom msg;
  msg.request.isRadians = true;
  msg.request.targetClockwiseDegreesRotation = 0;
  msg.request.targetClockwiseRadiansRotation = changeInRad;
  msg.request.targetDistance = distance;
  msg.request.targetSpeed = velocity;

  std::atomic<bool> runThread{true};
  std::thread rosLoopRunner = std::thread(rosSpinThread, std::ref(runThread));

  ASSERT_TRUE(odomClient.call(msg));

  // After n seconds we should stop turning
  testInstance.startTimers();
  std::chrono::duration<double> delayTime(timeForTurn * 1.2);
  std::this_thread::sleep_for(delayTime);

  // Stop thread calling ros::spin
  runThread = false;
  rosLoopRunner.join();

  auto targetSpeed = testInstance.getTargetSpeed();
  EXPECT_EQ(targetSpeed.leftWheel, velocity * METERS_TO_MILLIS);
  EXPECT_EQ(targetSpeed.rightWheel, velocity * METERS_TO_MILLIS);
}

TEST(Services_Odom, turnOnSpotHandlesBackwardsDirections) {
  ros::NodeHandle handle(HANDLE_PATH);
  Services testInstance(handle);

  ros::ServiceClient odomClient =
      handle.serviceClient<argo_driver::SetTargetOdom>('/' + HANDLE_PATH + '/' +
                                                       SET_ODOM_SERV_NAME);

  const double distance = 0;                // meter
  const double changeInRad = M_PI + M_PI_4; // or 225 degrees
  const double velocity = 10;               // m/s

  argo_driver::SetTargetOdom msg;
  msg.request.isRadians = true;
  msg.request.targetClockwiseDegreesRotation = 0;
  msg.request.targetClockwiseRadiansRotation = changeInRad;
  msg.request.targetDistance = distance;
  msg.request.targetSpeed = velocity;

  std::atomic<bool> runThread{true};
  std::thread rosLoopRunner = std::thread(rosSpinThread, std::ref(runThread));

  ASSERT_TRUE(odomClient.call(msg));

  runThread = false;
  rosLoopRunner.join();

  auto targetSpeed = testInstance.getTargetSpeed();
  EXPECT_EQ(targetSpeed.leftWheel, -TURN_SPEED * METERS_TO_MILLIS);
  EXPECT_EQ(targetSpeed.rightWheel, TURN_SPEED * METERS_TO_MILLIS);
}

TEST(Services_Odom, turnOnSpotHandlesAntiClockwise) {
  ros::NodeHandle handle(HANDLE_PATH);
  Services testInstance(handle);

  ros::ServiceClient odomClient =
      handle.serviceClient<argo_driver::SetTargetOdom>('/' + HANDLE_PATH + '/' +
                                                       SET_ODOM_SERV_NAME);

  const double distance = 0;                         // meter
  const double changeInRad = M_PI + M_PI_2 + M_PI_4; // or 315 degrees
  const double velocity = 10;                        // m/s

  argo_driver::SetTargetOdom msg;
  msg.request.isRadians = true;
  msg.request.targetClockwiseDegreesRotation = 0;
  msg.request.targetClockwiseRadiansRotation = changeInRad;
  msg.request.targetDistance = distance;
  msg.request.targetSpeed = velocity;

  std::atomic<bool> runThread{true};
  std::thread rosLoopRunner = std::thread(rosSpinThread, std::ref(runThread));

  ASSERT_TRUE(odomClient.call(msg));

  runThread = false;
  rosLoopRunner.join();

  auto targetSpeed = testInstance.getTargetSpeed();
  EXPECT_EQ(targetSpeed.leftWheel, -TURN_SPEED * METERS_TO_MILLIS);
  EXPECT_EQ(targetSpeed.rightWheel, TURN_SPEED * METERS_TO_MILLIS);
}

TEST(Services_Odom, turnHandlesReverse) {
  ros::NodeHandle handle(HANDLE_PATH);
  Services testInstance(handle);

  ros::ServiceClient odomClient =
      handle.serviceClient<argo_driver::SetTargetOdom>('/' + HANDLE_PATH + '/' +
                                                       SET_ODOM_SERV_NAME);

  const double distance = 1;                // meter
  const double changeInRad = M_PI + M_PI_4; // or 225 degrees
  const double velocity = 10;               // m/s

  // This should normalise to M_PI_4
  const int expectedVelocityDiff =
      calculateChangeInVelocity(M_PI_4, distance, velocity);

  argo_driver::SetTargetOdom msg;
  msg.request.isRadians = true;
  msg.request.targetClockwiseDegreesRotation = 0;
  msg.request.targetClockwiseRadiansRotation = changeInRad;
  msg.request.targetDistance = distance;
  msg.request.targetSpeed = velocity;

  std::atomic<bool> runThread{true};
  std::thread rosLoopRunner = std::thread(rosSpinThread, std::ref(runThread));
  ASSERT_TRUE(odomClient.call(msg));

  // Stop thread calling ros::spin
  runThread = false;
  rosLoopRunner.join();

  const double expectedLeftSpeed =
      (-velocity * METERS_TO_MILLIS) - expectedVelocityDiff;
  const double expectedRightSpeed =
      (-velocity * METERS_TO_MILLIS) + expectedVelocityDiff;

  auto targetSpeed = testInstance.getTargetSpeed();
  EXPECT_EQ(targetSpeed.leftWheel, expectedLeftSpeed);
  EXPECT_EQ(targetSpeed.rightWheel, expectedRightSpeed);
}

TEST(Services_Odom, turnHandlesAntiClockwise) {
  ros::NodeHandle handle(HANDLE_PATH);
  Services testInstance(handle);

  ros::ServiceClient odomClient =
      handle.serviceClient<argo_driver::SetTargetOdom>('/' + HANDLE_PATH + '/' +
                                                       SET_ODOM_SERV_NAME);

  const double distance = 1;                         // meter
  const double changeInRad = M_PI + M_PI_2 + M_PI_4; // or 315 degrees
  const double velocity = 10;                        // m/s

  // This should normalise to M_PI_4
  const int expectedVelocityDiff =
      calculateChangeInVelocity(M_PI_4, distance, velocity);

  argo_driver::SetTargetOdom msg;
  msg.request.isRadians = true;
  msg.request.targetClockwiseDegreesRotation = 0;
  msg.request.targetClockwiseRadiansRotation = changeInRad;
  msg.request.targetDistance = distance;
  msg.request.targetSpeed = velocity;

  std::atomic<bool> runThread{true};
  std::thread rosLoopRunner = std::thread(rosSpinThread, std::ref(runThread));
  ASSERT_TRUE(odomClient.call(msg));

  runThread = false;
  rosLoopRunner.join();

  const double expectedLeftSpeed =
      (velocity * METERS_TO_MILLIS) - expectedVelocityDiff;
  const double expectedRightSpeed =
      (velocity * METERS_TO_MILLIS) + expectedVelocityDiff;

  auto targetSpeed = testInstance.getTargetSpeed();
  EXPECT_EQ(targetSpeed.leftWheel, expectedLeftSpeed);
  EXPECT_EQ(targetSpeed.rightWheel, expectedRightSpeed);
}

TEST(Services_Stop, vehicleStopsOnCall) {
  ros::NodeHandle handle(HANDLE_PATH);
  Services testInstance(handle);

  ros::ServiceClient stopClient = handle.serviceClient<argo_driver::Stop>(
      '/' + HANDLE_PATH + '/' + STOP_SERV_NAME);

  // A blank message should stop the vehicle
  argo_driver::Stop msg;

  std::atomic<bool> runThread{true};
  std::thread rosLoopRunner = std::thread(rosSpinThread, std::ref(runThread));
  ASSERT_TRUE(stopClient.call(msg));

  runThread = false;
  rosLoopRunner.join();

  auto targetSpeed = testInstance.getTargetSpeed();
  EXPECT_EQ(targetSpeed.leftWheel, 0);
  EXPECT_EQ(targetSpeed.rightWheel, 0);
}