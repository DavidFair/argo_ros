#ifndef PUBLISHER_HPP_
#define PUBLISHER_HPP_

#include "ros/ros.h"

#include "CommsParser.hpp"

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

private:
  /// A handle to the left encoder count publisher
  ros::Publisher m_leftEncoderPub;
  /// A handle to the right encoder count publisher
  ros::Publisher m_rightEncoderPub;

  /// A handle to the left speed publisher
  ros::Publisher m_leftSpeedPub;
  /// A handle to the right speed publisher
  ros::Publisher m_rightSpeedPub;
};

#endif // PUBLISHER_HPP_