#ifndef ARGO_DRIVER_HPP_
#define ARGO_DRIVER_HPP_

#include "ros/ros.h"

#include "Parser.hpp"
#include "Publisher.hpp"
#include "SerialInterface.hpp"
#include "Services.hpp"

class ArgoDriver {
public:
  ArgoDriver(SerialInterface &commsObj, ros::NodeHandle &nodeHandle);

  void loop(const ros::TimerEvent &);
  void setup();

private:
  void parseCommand(CommandType type, const std::string &s);

  ros::NodeHandle &m_node;
  ros::Timer m_loopTimer;

  Publisher m_publisher;
  SerialInterface &m_serial;
  Services m_services;
};

#endif