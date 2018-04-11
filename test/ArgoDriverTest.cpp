#include <gtest/gtest.h>

#include "ros/ros.h"
#include "std_msgs/Int16.h"

#include "ArgoDriver.hpp"
#include "SerialCommsMock.hpp"
#include "SerialInterface.hpp"

using ::testing::NiceMock;
using ::testing::Return;
using ::testing::Test;
using ::testing::_;

namespace {
class ArgoDriverFixture : public ::testing::Test {
protected:
  ArgoDriverFixture()
      : handle(), mockComms(),
        testInstance(static_cast<SerialInterface &>(mockComms), handle) {}

  ros::NodeHandle handle;
  NiceMock<SerialCommsMock> mockComms;
  ArgoDriver testInstance;
};
} // End of anonymous namespace

TEST_F(ArgoDriverFixture, setupOpensSerial) {
  EXPECT_CALL(mockComms, openPort(_, _)).Times(1);
  testInstance.setup();
}

TEST_F(ArgoDriverFixture, loopCallsReadOnce) {
  std::string emptyString;
  EXPECT_CALL(mockComms, read()).WillOnce(Return(emptyString));

  ros::TimerEvent fakeEvent;
  testInstance.loop(fakeEvent);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "argo_driver_tests");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}