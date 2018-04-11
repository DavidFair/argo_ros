#include <sstream>

#include "ros/ros.h"
#include "std_msgs/Int16.h"

#include "Publisher.hpp"

namespace {
// Only allow the last 10 messages to queue before they get stale
const size_t TOPIC_QUEUE = 10;

// Topic Names:
const std::string L_ENC_TOPIC_NAME{"left_encoder_count"};
const std::string R_ENC_TOPIC_NAME{"right_encoder_count"};

const std::string L_SPEED_TOPIC_NAME{"left_speed"};
const std::string R_SPEED_TOPIC_NAME{"right_speed"};

} // namespace

Publisher::Publisher(ros::NodeHandle &handle)
    : m_handle(handle), m_leftEncoderPub(), m_rightEncoderPub(),
      m_leftSpeedPub(), m_rightSpeedPub() {

  // Create publishers - first encoder outputs
  m_leftEncoderPub =
      m_handle.advertise<std_msgs::Int16>(L_ENC_TOPIC_NAME, TOPIC_QUEUE);
  m_rightEncoderPub =
      m_handle.advertise<std_msgs::Int16>(R_ENC_TOPIC_NAME, TOPIC_QUEUE);

  m_leftSpeedPub =
      m_handle.advertise<std_msgs::Int16>(L_SPEED_TOPIC_NAME, TOPIC_QUEUE);
  m_rightSpeedPub =
      m_handle.advertise<std_msgs::Int16>(R_SPEED_TOPIC_NAME, TOPIC_QUEUE);
}

void Publisher::publishCurrentSpeed(const SpeedData &data) {
  if (!data.isValid) {
    return;
  }

  std_msgs::Int16 leftMessage;
  leftMessage.data = data.leftWheel;

  std_msgs::Int16 rightMessage;
  rightMessage.data = data.rightWheel;

  m_leftSpeedPub.publish(leftMessage);
  m_rightSpeedPub.publish(rightMessage);
}

void Publisher::publishEncoderCount(const EncoderData &data) {
  if (!data.isValid) {
    return;
  }

  std_msgs::Int16 leftMessage;
  leftMessage.data = data.leftWheel;

  std_msgs::Int16 rightMessage;
  rightMessage.data = data.rightWheel;

  m_leftEncoderPub.publish(leftMessage);
  m_rightEncoderPub.publish(rightMessage);
}