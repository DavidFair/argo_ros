#include <gtest/gtest.h>

#include "ros/ros.h"
#include "std_msgs/Int16.h"

#include "ArgoDriver.hpp"
#include "SerialInterface.hpp"
#include "SerialCommsMock.hpp"

using ::testing::NiceMock;
using ::testing::Return;
using ::testing::Test;
using ::testing::_;

namespace
{
// Based on https://answers.ros.org/question/195054/how-to-verify-publications-using-rostest/
// ----------------------------------
class TestSubscriber
{
  public:
    TestSubscriber() = default;
    void callback(const std_msgs::Int16ConstPtr &msg)
    {
        messageArrived = true;
        message = msg;
    }

    bool messageArrived{false};
    std_msgs::Int16ConstPtr message;
};

// --------------------------------

class ArgoDriverFixture : public ::testing::Test
{
  protected:
    ArgoDriverFixture() : handle(), mockComms(),
                          testInstance(static_cast<SerialInterface &>(mockComms), handle) {}

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
    std::string emptyString;
    EXPECT_CALL(mockComms, read()).WillOnce(Return(emptyString));

    ros::TimerEvent fakeEvent;
    testInstance.loop(fakeEvent);
}

TEST_F(ArgoDriverFixture, speedIsPublished)
{
    const int leftSpeed = 1234;
    const int rightSpeed = 2345;

    const std::string input =
        "!s LEFT:" + std::to_string(leftSpeed) + " RIGHT:" + std::to_string(rightSpeed) + '\n';

    ON_CALL(mockComms, read()).WillByDefault(Return(input));
    TestSubscriber leftListener;
    TestSubscriber rightListener;

    ros::Subscriber rosLeftListener = handle.subscribe("left_speed", 1,
                                                       &TestSubscriber::callback, &leftListener);
    ros::Subscriber rosRightListener = handle.subscribe("right_speed", 1,
                                                        &TestSubscriber::callback, &rightListener);

    testInstance.loop(ros::TimerEvent{});
    ros::spinOnce();

    EXPECT_TRUE(leftListener.messageArrived);
    EXPECT_TRUE(rightListener.messageArrived);

    ASSERT_TRUE(leftListener.message);
    ASSERT_TRUE(rightListener.message);

    EXPECT_EQ(leftListener.message->data, leftSpeed);
    EXPECT_EQ(rightListener.message->data, rightSpeed);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "argo_driver_tests");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}