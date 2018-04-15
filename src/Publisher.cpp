#include <cmath>
#include <sstream>

#include "ros/ros.h"
#include "std_msgs/Int16.h"

#include "ArgoGlobals.hpp"
#include "Publisher.hpp"
#include "argo_driver/CurrentOdom.h"

namespace {
// Only allow the last 10 messages to queue before they get stale
const size_t TOPIC_QUEUE = 10;

// Topic Names:
const std::string L_ENC_TOPIC_NAME{"left_encoder_count"};
const std::string R_ENC_TOPIC_NAME{"right_encoder_count"};

const std::string L_SPEED_TOPIC_NAME{"left_speed"};
const std::string R_SPEED_TOPIC_NAME{"right_speed"};

const std::string ODOM_TOPIC_NAME{"current_odom"};

/// Converts radians to degrees
double convertRadiansToDegrees(double radians) {
  return radians * (180 / M_PI);
}

} // namespace

/**
 * Creates a new publisher object which in turn
 * creates ROS topics with the referenced node handle
 *
 * @param handle The ROS node to create publishers for
 */
Publisher::Publisher(ros::NodeHandle &handle)
    : m_leftEncoderPub(), m_rightEncoderPub(), m_leftSpeedPub(),
      m_rightSpeedPub() {
  m_leftEncoderPub =
      handle.advertise<std_msgs::Int16>(L_ENC_TOPIC_NAME, TOPIC_QUEUE);
  m_rightEncoderPub =
      handle.advertise<std_msgs::Int16>(R_ENC_TOPIC_NAME, TOPIC_QUEUE);

  m_odomPub =
      handle.advertise<argo_driver::CurrentOdom>(ODOM_TOPIC_NAME, TOPIC_QUEUE);

  m_leftSpeedPub =
      handle.advertise<std_msgs::Int16>(L_SPEED_TOPIC_NAME, TOPIC_QUEUE);
  m_rightSpeedPub =
      handle.advertise<std_msgs::Int16>(R_SPEED_TOPIC_NAME, TOPIC_QUEUE);
}

/**
 * Publishes the current vehicle speed to the associated topics
 * from the input data. (See SpeedData).
 *
 * @param data The vehicle speed to publish
 */
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

/**
 * Publishes the current odometry, such as distance travelled in/by the:
 * x, y, leftWheels, rightWheels, currentHeading. Based off the current
 * encoder data. (See EncoderData)
 *
 * @param data The encoder data to calculate the attributes from
 */
void Publisher::publishCurrentOdometry(const EncoderData &data) {
  if (!data.isValid) {
    return;
  }

  const double leftWheelDistance = data.leftWheel * g_DIST_PER_ENC_COUNT;
  const double rightWheelDistance = data.rightWheel * g_DIST_PER_ENC_COUNT;
  const double difference = leftWheelDistance - rightWheelDistance;

  // In radians
  double currentHeading = difference / g_LENGTH_BETWEEN_WHEELS;
  if (currentHeading >= 2 * M_PI) {
    // Normalise to within 2PI (or 360 deg)
    currentHeading = fmod(currentHeading, 2 * M_PI);
  }

  // Get the smallest scalar distance travelled
  const double baseDistTravelled = (leftWheelDistance + rightWheelDistance) / 2;
  const double xDist = baseDistTravelled * cos(currentHeading);
  const double yDist = baseDistTravelled * sin(currentHeading);

  argo_driver::CurrentOdom msg;

  msg.headingDegrees = convertRadiansToDegrees(currentHeading);
  msg.headingRadians = currentHeading;
  msg.leftWheelTravelled = leftWheelDistance;
  msg.rightWheelTravelled = rightWheelDistance;
  msg.xDistTravelled = xDist;
  msg.yDistTravelled = yDist;

  m_odomPub.publish(msg);
}

/**
 * Publishes the current encoder counts which are absolute from
 * the input data. (See EncoderData).
 *
 * @param data The encoder counts to publish
 */
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
