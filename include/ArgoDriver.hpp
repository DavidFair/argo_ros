#ifndef ARGO_DRIVER_HPP_
#define ARGO_DRIVER_HPP_

#include "ros/ros.h"

#include "SerialInterface.hpp"

class ArgoDriver {
public:
  ArgoDriver(SerialInterface &commsObj, ros::NodeHandle &nodeHandle);

  void loop(const ros::TimerEvent &event);
  void setup();

private:
  ros::NodeHandle &m_node;
  ros::Timer m_loopTimer;

  SerialInterface &m_serial;
};

#endif