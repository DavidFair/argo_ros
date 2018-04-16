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

/**
 * Implements the ROS node for controlling the Argo
 */
class ArgoDriver {
public:
  /// Constructs a new driver node that runs the main loop
  ArgoDriver(SerialInterface &commsObj, ros::NodeHandle &nodeHandle,
             bool timeoutEnabled = true);

  /// The main loop that is executed by the ROS node
  void loop(const ros::TimerEvent &);

  /// Sets up and fires the main loop
  void setup();

private:
  /// Sends a ping to the vehicle and checks the incoming ping has
  /// not timed out
  bool exchangePing();

  /// Limits the current given speed data to the maximum velocity
  bool limitToMaxVelocity(SpeedData &values) const;

  /// Parses the given command of type and performs appropriate actions
  void parseCommand(CommandType type, const std::string &s);

  /// Reads any communications and processes each
  void readFromArduino();

  /// Appends a new target speed, if it has changed, to the outgoing buffer
  void updateTargetSpeed(SpeedData currentSpeedTarget);

  // *ROS member variables*

  /// Node handle for ROS
  ros::NodeHandle &m_node;
  /// Timer the fires subseqent loop iterations
  ros::Timer m_loopTimer;

  // *Internal variables*
  /// The maximum allowed velocity of the vehicle
  const int m_maxVelocity; // TODO
  /// The current speed the vehicle should be traveling at
  SpeedData m_previousSpeedData;
  /// The last time the speed command was issued
  std::chrono::time_point<std::chrono::steady_clock> m_lastSpeedCommandTime;
  /// The pending buffer to write to the serial port
  std::vector<std::string> m_outputBuffer;

  // *Ping related variables*
  /// Indicates whether pings are sent and used to timeout the node
  const bool m_usePings;
  /// Tracks whether any comms have been heard yet
  bool m_commsHasBeenMade;

  /// The time that the last incoming ping was received
  std::chrono::time_point<std::chrono::steady_clock> m_lastIncomingPingTime;

  // * Helper classes *
  /// An object which handles publishing in ROS
  Publisher m_publisher;
  /// An object which handles communications to the vehicle
  SerialInterface &m_serial;
  /// An object which provides the nodes services
  Services m_services;
};

#endif