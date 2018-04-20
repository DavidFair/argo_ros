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
#include "argo_driver/Wheels.h"

namespace {

const std::string nodePath = "ArgoPublisherTests";
const std::string speedTopicName = "current_speed";
const std::string encoderTopicName = "current_encoder";
const std::string pwmTopicName = "current_pwm";

template <typename msgT> class rosSubscriber {
public:
  rosSubscriber(ros::NodeHandle &node, const std::string &topicName)
      : m_subscriber(node.subscribe(topicName, 1,
                                    &rosSubscriber<msgT>::rosCallback, this)) {}

  msgT getLastMsg() const { return receivedMsg; }

  bool m_callbackTriggered{false};

private:
  void rosCallback(const msgT incomingMsg) {
    m_callbackTriggered = true;
    receivedMsg = incomingMsg;
  }

  ros::Subscriber m_subscriber;
  msgT receivedMsg;
};

void rosSpinAndSleep() {
  using namespace std::chrono_literals;
  for (int i = 0; i < 2; i++) {
    std::this_thread::sleep_for(50ms);
    ros::spinOnce();
  }
}

} // Anonymous namespace

TEST(Publisher, publishesCurrentSpeed) {
  ros::NodeHandle handle(nodePath);
  Publisher testInstance(handle);

  const int targetLeftSpeed = 1000;  // millis per second
  const int targetRightSpeed = 2000; // millis per second

  SpeedData targetSpeeds{targetLeftSpeed, targetRightSpeed};

  rosSubscriber<argo_driver::Wheels> subTester(handle, speedTopicName);
  testInstance.publishCurrentSpeed(targetSpeeds);

  rosSpinAndSleep();
  ASSERT_TRUE(subTester.m_callbackTriggered);

  auto receivedMsg = subTester.getLastMsg();
  EXPECT_EQ(targetLeftSpeed / MILLIS_PER_SEC, receivedMsg.leftWheel);
  EXPECT_EQ(targetRightSpeed / MILLIS_PER_SEC, receivedMsg.rightWheel);
}

TEST(Publisher, publishesEncoderCount) {
  ros::NodeHandle handle(nodePath);
  Publisher testInstance(handle);

  const int leftEncCount = 123;
  const int rightEncCount = 234;

  rosSubscriber<argo_driver::Wheels> callback(handle, encoderTopicName);

  EncoderData data{leftEncCount, rightEncCount};
  testInstance.publishEncoderCount(data);

  rosSpinAndSleep();
  ASSERT_TRUE(callback.m_callbackTriggered);

  auto receivedMsg = callback.getLastMsg();
  EXPECT_EQ(leftEncCount, receivedMsg.leftWheel);
  EXPECT_EQ(rightEncCount, receivedMsg.rightWheel);
}

TEST(Publisher, publishesPwmValues) {
  ros::NodeHandle handle(nodePath);
  Publisher testInstance(handle);

  const int leftPwmVal = 100;
  const int rightPwmVal = 200;

  rosSubscriber<argo_driver::Wheels> callback(handle, pwmTopicName);

  PwmData data{leftPwmVal, rightPwmVal};
  testInstance.publishPwmValues(data);

  rosSpinAndSleep();
  ASSERT_TRUE(callback.m_callbackTriggered);

  auto receivedMsg = callback.getLastMsg();
  EXPECT_EQ(leftPwmVal, receivedMsg.leftWheel);
  EXPECT_EQ(rightPwmVal, receivedMsg.rightWheel);
}

TEST(Publisher, publishesTargetSpeed) {
  ros::NodeHandle handle(nodePath);
  Publisher testInstance(handle);

  const int targetLeftSpeed = 2000;  // millis per second
  const int targetRightSpeed = 4000; // millis per second

  SpeedData targetSpeeds{targetLeftSpeed, targetRightSpeed};

  rosSubscriber<argo_driver::Wheels> subTester(handle, speedTopicName);
  testInstance.publishCurrentSpeed(targetSpeeds);

  rosSpinAndSleep();
  ASSERT_TRUE(subTester.m_callbackTriggered);

  auto receivedMsg = subTester.getLastMsg();
  EXPECT_EQ(targetLeftSpeed / MILLIS_PER_SEC, receivedMsg.leftWheel);
  EXPECT_EQ(targetRightSpeed / MILLIS_PER_SEC, receivedMsg.rightWheel);
}