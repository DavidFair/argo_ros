#include <cmath>
#include <sstream>
#include <utility>

#include "ros/ros.h"
#include "tf/transform_datatypes.h"

#include "ArgoGlobals.hpp"
#include "CommsParser.hpp"
#include "Publisher.hpp"
#include "argo_driver/Wheels.h"
#include "nav_msgs/Odometry.h"

namespace {
// Only allow the last 10 messages to queue before they get stale
const size_t TOPIC_QUEUE = 10;

// Topic Names:

const std::string TARGET_SPEED{"target_speed"};
const std::string ODOM_TOPIC_NAME{"odom"};

const std::string ENC_TOPIC_NAME{"current_encoder"};
const std::string CURRENT_SPEED{"current_speed"};
const std::string PWM_TOPIC_NAME{"current_pwm"};
} // namespace

/**
 * Creates a new publisher object which in turn
 * creates ROS topics with the referenced node handle
 *
 * @param handle The ROS node to create publishers for
 */
Publisher::Publisher(ros::NodeHandle &handle)
    : m_currentSpeedPub(), m_targetSpeedPub(), m_odomPub(), m_pwmPub(),
      m_encoderPub() {
  m_currentSpeedPub =
      handle.advertise<argo_driver::Wheels>(CURRENT_SPEED, TOPIC_QUEUE);
  m_targetSpeedPub =
      handle.advertise<argo_driver::Wheels>(TARGET_SPEED, TOPIC_QUEUE);

  m_odomPub =
      handle.advertise<nav_msgs::Odometry>(ODOM_TOPIC_NAME, TOPIC_QUEUE);

  m_pwmPub = handle.advertise<argo_driver::Wheels>(PWM_TOPIC_NAME, TOPIC_QUEUE);

  m_encoderPub =
      handle.advertise<argo_driver::Wheels>(ENC_TOPIC_NAME, TOPIC_QUEUE);
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

  argo_driver::Wheels msg;
  msg.leftWheel = (double)data.leftWheel / METERS_TO_MILLIS;
  msg.rightWheel = (double)data.rightWheel / METERS_TO_MILLIS;

  m_currentSpeedPub.publish(std::move(msg));
}

/**
 * Publishes the current odometry, such as distance travelled in/by the:
 * x, y, leftWheels, rightWheels, currentHeading. Based off the current
 * encoder data. (See EncoderData)
 *
 * @param data The encoder data to calculate the attributes from
 */
void Publisher::publishCurrentOdometry(const EncoderData &encoder,
                                       const SpeedData &speed) {
  if (!encoder.isValid || !speed.isValid) {
    return;
  }

  const double leftWheelDistance = encoder.leftWheel * g_DIST_PER_ENC_COUNT;
  const double rightWheelDistance = encoder.rightWheel * g_DIST_PER_ENC_COUNT;
  const double difference = leftWheelDistance - rightWheelDistance;

  // In radians
  double currentHeading = difference / g_LENGTH_BETWEEN_WHEELS;
  if (currentHeading >= 2 * M_PI || currentHeading <= 2 * M_PI) {
    // Normalise to within 2PI (or 360 deg)
    currentHeading = fmod(currentHeading, 2 * M_PI);
  }

  // Get the smallest scalar distance travelled
  const double baseDistTravelled = (leftWheelDistance + rightWheelDistance) / 2;
  const double xDist = baseDistTravelled * cos(currentHeading);
  const double yDist = baseDistTravelled * sin(currentHeading);

  // ------- Adapted from example at -------
  // http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom#Publishing_Odometry_Information_Over_ROS
  geometry_msgs::Quaternion quaternion =
      tf::createQuaternionMsgFromYaw(currentHeading);

  nav_msgs::Odometry odomMsg;
  odomMsg.header.stamp = ros::Time::now();
  odomMsg.header.frame_id = "odom";

  odomMsg.pose.pose.position.x = xDist;
  odomMsg.pose.pose.position.y = yDist;
  odomMsg.pose.pose.position.z = 0.;
  odomMsg.pose.pose.orientation = quaternion;

  // Take speed closest to 0 as base speed
  const int baseSpeed = std::abs(speed.leftWheel) < std::abs(speed.rightWheel)
                            ? speed.leftWheel
                            : speed.rightWheel;
  const int velDiff = speed.leftWheel - speed.rightWheel;

  odomMsg.twist.twist.linear.x = baseSpeed / (double)METERS_TO_MILLIS;
  odomMsg.twist.twist.linear.y = 0.; // Argo cannot translate through Y
  odomMsg.twist.twist.angular.z = velDiff / (double)METERS_TO_MILLIS;

  // ---------------------
  m_odomPub.publish(std::move(odomMsg));
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

  argo_driver::Wheels msg;
  msg.leftWheel = data.leftWheel;
  msg.rightWheel = data.rightWheel;

  m_encoderPub.publish(std::move(msg));
}

/**
 * Publishes the current PWM values from the input data. (See PwmValues).
 *
 * @param data The PWM values to publish
 */
void Publisher::publishPwmValues(const PwmData &vals) {
  if (!vals.isValid) {
    return;
  }

  argo_driver::Wheels msg;
  msg.leftWheel = vals.leftWheel;
  msg.rightWheel = vals.rightWheel;

  m_pwmPub.publish(std::move(msg));
}

/**
 * Publishes the current target speed in meters per second which is passed
 * as a parameter. (See SpeedData)
 *
 * @param data Target speed to publish
 */
void Publisher::publishTargetSpeed(const SpeedData &data) {

  argo_driver::Wheels msg;
  msg.leftWheel = (double)data.leftWheel / METERS_TO_MILLIS;
  msg.rightWheel = (double)data.rightWheel / METERS_TO_MILLIS;

  m_targetSpeedPub.publish(std::move(msg));
}