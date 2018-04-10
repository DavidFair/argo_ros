#ifndef PUBLISHER_HPP_
#define PUBLISHER_HPP_

#include "ros/ros.h"

class Publisher {
public:
  Publisher(ros::NodeHandle &handle);
  void parseIncomingBuffer(const std::string &received);

private:
  void determineCommandType(const std::string &input);

  void parseEncoderOutput(const std::string &input);
  void parseSpeedOutput(const std::string &input);

  void publishEncoderCount(const int leftCount, const int rightCount);
  void publishCurrentSpeed(const int leftWheels, const int rightWheels);

  ros::NodeHandle &m_handle;
};

#endif // PUBLISHER_HPP_