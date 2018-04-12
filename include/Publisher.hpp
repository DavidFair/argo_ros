#ifndef PUBLISHER_HPP_
#define PUBLISHER_HPP_

#include "ros/ros.h"

#include "CommsParser.hpp"

class Publisher {
public:
  Publisher(ros::NodeHandle &handle);

  void publishEncoderCount(const EncoderData &data);
  void publishCurrentSpeed(const SpeedData &data);

private:
  ros::Publisher m_leftEncoderPub;
  ros::Publisher m_rightEncoderPub;

  ros::Publisher m_leftSpeedPub;
  ros::Publisher m_rightSpeedPub;
};

#endif // PUBLISHER_HPP_