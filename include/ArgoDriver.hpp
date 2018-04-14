#ifndef ARGO_DRIVER_HPP_
#define ARGO_DRIVER_HPP_

#include <chrono>
#include <string>
#include <vector>

#include "ros/ros.h"

#include "CommsParser.hpp"
#include "Publisher.hpp"
#include "SerialInterface.hpp"
#include "Services.hpp"

class ArgoDriver {
public:
  ArgoDriver(SerialInterface &commsObj, ros::NodeHandle &nodeHandle,
             bool timeoutEnabled = true);

  void loop(const ros::TimerEvent &);
  void setup();

private:
  bool exchangePing();
  void parseCommand(CommandType type, const std::string &s);
  void readFromArduino();
  void updateTargetSpeed();

  // ROS member variables
  ros::NodeHandle &m_node;
  ros::Timer m_loopTimer;

  // Internal variables
  const int m_maxVelocity;
  SpeedData m_previousSpeedData;
  std::vector<std::string> m_outputBuffer;

  // Ping related variables
  bool m_usePings;
  std::chrono::time_point<std::chrono::steady_clock> m_lastIncomingPingTime;
  std::chrono::time_point<std::chrono::steady_clock> m_lastOutgoingPingTime;

  // Helper classes
  Publisher m_publisher;
  SerialInterface &m_serial;
  Services m_services;
};

#endif