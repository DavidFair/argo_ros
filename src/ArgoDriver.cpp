#include <chrono>
#include <string>

#include "ros/ros.h"

#include "ArgoDriver.hpp"
#include "CommsParser.hpp"
#include "Publisher.hpp"
#include "SerialInterface.hpp"

namespace {
using namespace std::chrono_literals;

// ROS defaults
const std::string DEFAULT_TTY = "/dev/ttyACM0";
const double DEFAULT_MAX_VEL = 3; // Meters per second

const double MILLIS_PER_METER = 1000;
const double MILLIS_PER_SEC = 1000;

const double LOOP_TIMER = 100;       // Time in ms per loop
const auto TIMEOUT_DURATION = 500ms; // Ping timeout

} // Anonymous namespace

ArgoDriver::ArgoDriver(SerialInterface &commsObj, ros::NodeHandle &nodeHandle,
                       bool useTimeouts)
    : m_node(nodeHandle),
      m_maxVelocity(nodeHandle.param<double>("maxVelocity", DEFAULT_MAX_VEL) *
                    MILLIS_PER_METER),
      m_previousSpeedData(), m_usePings(useTimeouts),
      m_lastIncomingPingTime(std::chrono::steady_clock::now()),
      m_lastOutgoingPingTime(std::chrono::steady_clock::now()),
      m_publisher(nodeHandle), m_serial(commsObj), m_services(nodeHandle) {}

void ArgoDriver::loop(const ros::TimerEvent &event) {
  // Prevent the timer from firing again until we are finished
  m_loopTimer.stop();

  bool enterNextLoop = true;

  readFromArduino();

  updateTargetSpeed();

  enterNextLoop &= exchangePing();

  if (m_serial.write(m_outputBuffer)) {
    m_outputBuffer.clear();
  } else {
    ROS_WARN("Failed to write output buffer, trying again");
  }

  // Start any pending timers if there are any
  m_services.startTimers();

  if (ros::ok() && enterNextLoop) {
    m_loopTimer.start();
  } else {
    ros::shutdown();
  }
}

void ArgoDriver::setup() {
  // Get the target tty port
  int baudRate = 0;
  std::string targetPort;
  m_node.param("ttyPort", targetPort, DEFAULT_TTY);
  m_node.param("baudRate", baudRate, 115200);

  // Setup serial comms to device
  m_serial.openPort(targetPort, baudRate);

  // Setup main loop to fire
  const bool isOneShot = false;
  m_loopTimer = m_node.createTimer(ros::Duration(LOOP_TIMER / 1000),
                                   &ArgoDriver::loop, this, isOneShot);
}

// ------ Private methods --------:

bool ArgoDriver::exchangePing() {
  if (!m_usePings) {
    // For unit testing where we don't want to timeout
    return true;
  }

  m_outputBuffer.push_back(CommsParser::getPingCommand());

  // Check if we have exceeded the maximum ping time yet
  auto currentTime = std::chrono::steady_clock::now();
  auto duration = currentTime - m_lastIncomingPingTime;
  if (duration > TIMEOUT_DURATION) {
    ROS_FATAL("Arduino has not responded in timeout. Entering deadman mode.");
    m_outputBuffer.clear();
    m_outputBuffer.push_back(CommsParser::getDeadmanCommand());
    return false;
  }
  return true;
}

void ArgoDriver::parseCommand(CommandType type, const std::string &s) {
  switch (type) {
  case CommandType::None:
    return;
  case CommandType::Encoder: {
    auto encoderData = CommsParser::parseEncoderCommand(s);
    m_publisher.publishEncoderCount(encoderData);
    break;
  }
  case CommandType::Fatal: {
    const std::string out{"Got fatal warning from Arduino:\n" + s};
    ROS_FATAL(out.c_str());
    ros::shutdown();
  }
  case CommandType::Ping: {
    ROS_DEBUG("Received ping");
    m_lastIncomingPingTime = std::chrono::steady_clock::now();
  }
  case CommandType::Speed: {
    auto speedData = CommsParser::parseSpeedCommand(s);
    m_publisher.publishCurrentSpeed(speedData);
    break;
  }
  case CommandType::Warning: {
    const std::string out{"Got warning from Arduino:\n" + s};
    ROS_WARN(out.c_str());
  }
  } // End of switch
}

void ArgoDriver::readFromArduino() {
  // Read from Arduino
  const auto serialInput = m_serial.read();
  for (const auto &incomingString : serialInput) {
    auto commandType = CommsParser::parseIncomingBuffer(incomingString);
    parseCommand(commandType, incomingString);
  }
}

void ArgoDriver::updateTargetSpeed() {
  // Get our current target speed and send an update if required
  auto currentSpeedTarget = m_services.getTargetSpeed();
  if (!(currentSpeedTarget == m_previousSpeedData)) {
    m_previousSpeedData = currentSpeedTarget;
    m_outputBuffer.push_back(CommsParser::getSpeedCommand(currentSpeedTarget));
  }
}