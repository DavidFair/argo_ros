#include <atomic>
#include <chrono>
#include <cmath>
#include <thread>

#include "ros/ros.h"
#include "gtest/gtest.h"

#include "ArgoGlobals.hpp"
#include "CommsParser.hpp"
#include "Publisher.hpp"
#include "argo_driver/CurrentOdom.h"

namespace {

class odomSubscriber {
public:
  odomSubscriber()
      : m_handle("PublisherTest"),
        m_subscriber(m_handle.subscribe("/PublisherTest/current_odom", 1,
                                        &odomSubscriber::callback, this)) {}

  void callback(const argo_driver::CurrentOdom::ConstPtr &msg) {
    callbackTriggered = true;
    receivedMsg = msg;
  }

  bool callbackTriggered{false};
  argo_driver::CurrentOdom::ConstPtr receivedMsg{};
  ros::NodeHandle m_handle;

private:
  ros::Subscriber m_subscriber;
};

} // Anonymous namespace

TEST(PublishOdom, straightDrive) {
  const double expectedDist = 10;
  const int encoderCounts = expectedDist / g_DIST_PER_ENC_COUNT;

  EncoderData inputData{encoderCounts, encoderCounts};
  odomSubscriber newSub;

  Publisher testInstance(newSub.m_handle);
  testInstance.publishCurrentOdometry(inputData);

  using namespace std::chrono_literals;
  const int MAX_SPINS = 5;
  int currentSpins = 0;

  while (!newSub.callbackTriggered && currentSpins < MAX_SPINS) {
    std::this_thread::sleep_for(100ms);
    ros::spinOnce();
  }

  ASSERT_TRUE(newSub.callbackTriggered);

  const argo_driver::CurrentOdom &msg = *(newSub.receivedMsg.get());

  const double tolerance = 0.01; // As the definition is only precise to 2DP
  EXPECT_NEAR(msg.leftWheelTravelled, expectedDist, tolerance);
  EXPECT_NEAR(msg.rightWheelTravelled, expectedDist, tolerance);

  EXPECT_EQ(msg.headingDegrees, 0);
  EXPECT_EQ(msg.headingRadians, 0);

  EXPECT_NEAR(msg.xDistTravelled, expectedDist, tolerance);
  EXPECT_NEAR(msg.yDistTravelled, 0, tolerance);
}

TEST(PublishOdom, onBearing) {
  const double expectedAngle = M_PI_4; // Or 45 degrees
  const double difference = expectedAngle * g_LENGTH_BETWEEN_WHEELS;

  const double expectedDist = 10; // meters

  const int leftEncCount = (expectedDist + difference) / g_DIST_PER_ENC_COUNT;
  const int rightEncCount = expectedDist / g_DIST_PER_ENC_COUNT;

  EncoderData inputData{leftEncCount, rightEncCount};

  odomSubscriber newSub;
  Publisher testInstance(newSub.m_handle);
  testInstance.publishCurrentOdometry(inputData);

  using namespace std::chrono_literals;
  const int MAX_SPINS = 5;
  int currentSpins = 0;

  while (!newSub.callbackTriggered && currentSpins < MAX_SPINS) {
    std::this_thread::sleep_for(100ms);
    ros::spinOnce();
  }

  ASSERT_TRUE(newSub.callbackTriggered);

  const argo_driver::CurrentOdom &msg = *(newSub.receivedMsg.get());

  const double tolerance = 0.01; // As the definition is only precise to 2DP
  EXPECT_NEAR(msg.leftWheelTravelled, (expectedDist + difference), tolerance);
  EXPECT_NEAR(msg.rightWheelTravelled, expectedDist, tolerance);

  EXPECT_EQ(msg.headingDegrees, 45);
  EXPECT_NEAR(msg.headingRadians, M_PI_4, 0.001); // 3DP for radians

  EXPECT_NEAR(msg.xDistTravelled, msg.yDistTravelled, tolerance);
}