#ifndef PUBLISHER_HPP_
#define PUBLISHER_HPP_

#include "ros/ros.h"

// Forward declerations
struct EncoderData;
struct PwmData;
struct SpeedData;

class Publisher {
public:
  /// Creates an object which handles publishing topics for the Argo driver
  Publisher(ros::NodeHandle &handle);

  /// Publishes the current vehicle speed
  void publishCurrentSpeed(const SpeedData &data);

  /// Publishes the current vehicle odometry
  void publishCurrentOdometry(const EncoderData &data);

  /// Publishes the current encoder count on the vehicle
  void publishEncoderCount(const EncoderData &data);

  /// Publishes the current PWM values on the vehicle
  void publishPwmValues(const PwmData &data);

  /// Publishes the target vehicle speed
  void publishTargetSpeed(const SpeedData &data);

private:
  /// A handle to publish the vehicles current speed
  ros::Publisher m_currentSpeedPub;

  /// A handle to publish the vehicles target speed
  ros::Publisher m_targetSpeedPub;

  /// A handle to publish the current odometry
  ros::Publisher m_odomPub;

  /// A handle to publish the current PWM values
  ros::Publisher m_pwmPub;

  /// A handle to publish the current encoder counts
  ros::Publisher m_encoderPub;
};

#endif // PUBLISHER_HPP_