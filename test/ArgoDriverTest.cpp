#include <atomic>
#include <chrono>
#include <thread>

#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "gtest/gtest.h"

#include "ArgoDriver.hpp"
#include "SerialCommsMock.hpp"
#include "SerialInterface.hpp"

using ::testing::AnyNumber;
using ::testing::NiceMock;
using ::testing::Return;
using ::testing::Test;
using ::testing::_;

namespace {
using namespace std::chrono_literals;
const std::string HANDLE_PATH{"argo_driver_test"};
const auto TIMEOUT = 250ms + 1ms;

const bool disablePings = false;
class ArgoDriverFixture : public ::testing::Test {
protected:
  ArgoDriverFixture()
      : handle(HANDLE_PATH), mockComms(),
        testInstance(static_cast<SerialInterface &>(mockComms), handle,
                     disablePings) {}

  ros::NodeHandle handle;
  NiceMock<SerialCommsMock> mockComms;
  ArgoDriver testInstance;
};

void spinRos(std::atomic<bool> &keepSpinning) {
  while (keepSpinning) {
    ros::spinOnce();
  }
}

} // End of anonymous namespace

TEST_F(ArgoDriverFixture, setupOpensSerial) {
  EXPECT_CALL(mockComms, openPort(_, _)).Times(1);
  testInstance.setup();
}

TEST_F(ArgoDriverFixture, loopCallsReadOnce) {
  const std::string emptyString;
  EXPECT_CALL(mockComms, read()).WillOnce(Return(emptyString));

  ros::TimerEvent fakeEvent;
  testInstance.loop(fakeEvent);
}

TEST_F(ArgoDriverFixture, newSpeedIsWrittenToSerial) {
  const std::string targetService = {"set_target_speed"};

  ros::ServiceClient speedClient =
      handle.serviceClient<argo_driver::SetWheelSpeeds>('/' + HANDLE_PATH +
                                                        '/' + targetService);

  argo_driver::SetWheelSpeeds msg;
  msg.request.leftWheelTarget = 1;
  msg.request.rightWheelTarget = 2;

  std::atomic<bool> runThread{true};
  std::thread rosLoopRunner = std::thread(spinRos, std::ref(runThread));

  ASSERT_TRUE(speedClient.call(msg));

  // Stop thread calling ros::spin
  runThread = false;
  rosLoopRunner.join();

  const std::string expectedString{"!T L_SPEED:1000 R_SPEED:2000\n"};
  EXPECT_CALL(mockComms, write(expectedString)).Times(1);

  testInstance.loop(ros::TimerEvent{});
}

TEST(ArgoDriverTimeout, pingIsSent) {
  ros::NodeHandle handle(HANDLE_PATH);
  NiceMock<SerialCommsMock> mockComms;
  const bool usePingTimeout = true;
  ArgoDriver testInstance(static_cast<SerialInterface &>(mockComms), handle,
                          usePingTimeout);

  // Expect a ping out only without triggering deadman
  EXPECT_CALL(mockComms, write("!P\n")).Times(1);
  testInstance.loop(ros::TimerEvent{});
}

TEST(ArgoDriverTimeout, pingTimeout) {
  ros::NodeHandle handle(HANDLE_PATH);
  NiceMock<SerialCommsMock> mockComms;
  const bool usePingTimeout = true;
  ArgoDriver testInstance(static_cast<SerialInterface &>(mockComms), handle,
                          usePingTimeout);

  // Expect a ping out first
  EXPECT_CALL(mockComms, write("!P\n")).Times(1).RetiresOnSaturation();

  EXPECT_CALL(mockComms, write("!D\n")).Times(1);
  std::this_thread::sleep_for(TIMEOUT);
  testInstance.loop(ros::TimerEvent{});
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "argo_driver_tests");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}