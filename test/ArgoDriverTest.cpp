#include <atomic>
#include <chrono>
#include <thread>
#include <gmock/gmock.h>

#include "ros/ros.h"
#include "std_msgs/Int16.h"

#include "ArgoDriver.hpp"
#include "SerialCommsMock.hpp"
#include "SerialInterface.hpp"
#include "argo_driver/Wheels.h"

using ::testing::AnyNumber;
using ::testing::Contains;
using ::testing::HasSubstr;
using ::testing::NiceMock;
using ::testing::Return;
using ::testing::StrictMock;
using ::testing::Test;
using ::testing::_;

namespace
{
using namespace std::chrono_literals;
const std::string HANDLE_PATH{"argo_driver_test"};
const auto TIMEOUT = 500ms + 1ms;

const bool disablePings = false;
class ArgoDriverFixture : public ::testing::Test
{
protected:
  ArgoDriverFixture()
      : handle(HANDLE_PATH), mockComms(),
        testInstance(static_cast<SerialInterface &>(mockComms), handle,
                     disablePings) {}

  ros::NodeHandle handle;
  NiceMock<SerialCommsMock> mockComms;
  ArgoDriver testInstance;
};

} // End of anonymous namespace

TEST_F(ArgoDriverFixture, setupOpensSerial)
{
  EXPECT_CALL(mockComms, openPort(_, _)).Times(1);
  testInstance.setup();
}

TEST_F(ArgoDriverFixture, loopCallsReadOnce)
{
  const std::vector<std::string> emptyString;
  EXPECT_CALL(mockComms, read()).WillOnce(Return(emptyString));

  ros::TimerEvent fakeEvent;
  testInstance.loop(fakeEvent);
}

TEST_F(ArgoDriverFixture, newSpeedIsWrittenToSerial)
{
  const std::string targetService = {"cmd_wheel_speeds"};

  const int MAX_TOPIC_LEN = 1;

  ros::Publisher speedClient = handle.advertise<argo_driver::Wheels>(
      '/' + HANDLE_PATH + '/' + targetService, MAX_TOPIC_LEN);

  argo_driver::Wheels msg;
  msg.leftWheel = 1;
  msg.rightWheel = 2;

  speedClient.publish(msg);

  for (int i = 0; i < 5; i++)
  {
    std::this_thread::sleep_for(50ms);
    ros::spinOnce();
  }

  const std::string expectedString{"!T L_SPEED:1000 R_SPEED:2000"};
  EXPECT_CALL(mockComms, write(Contains(HasSubstr(expectedString)))).Times(1);

  testInstance.loop(ros::TimerEvent{});
}

TEST(ArgoDriverTimeout, NothingIsSentAfterTimeout)
{
  ros::NodeHandle handle(HANDLE_PATH);
  StrictMock<SerialCommsMock> mockComms;
  const bool usePingTimeout = true;
  ArgoDriver testInstance(static_cast<SerialInterface &>(mockComms), handle,
                          usePingTimeout);
  EXPECT_CALL(mockComms, read()).Times(AnyNumber());

  testInstance.loop(ros::TimerEvent{});
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "argo_driver_tests");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}