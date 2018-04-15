#include <chrono>
#include <string>

#include "ros/ros.h"

#include "ArgoDriver.hpp"
#include "ArgoGlobals.hpp"
#include "CommsParser.hpp"
#include "Publisher.hpp"
#include "SerialInterface.hpp"

namespace {
using namespace std::chrono_literals;

// ROS defaults
const std::string DEFAULT_TTY = "/dev/ttyACM0";
const double DEFAULT_MAX_VEL = 3; // Meters per second

const double LOOP_TIMER = 100;       // Time in ms per loop
const auto TIMEOUT_DURATION = 500ms; // Ping timeout

} // Anonymous namespace

/**
 * Creates a new instance of the ROS driver
 *
 * @param SerialInterface Reference to a concrete implementation of the
 * SerialInterface for the node to communicate over
 * @param nodeHandle Reference to a setup ROS node handle
 * @param timeoutEnabled (Default True). If true enables sending pings
 * and entering deadman mode. Primarily used in unit tests to disable pings.
 */
ArgoDriver::ArgoDriver(SerialInterface &commsObj, ros::NodeHandle &nodeHandle,
                       bool useTimeouts)
    : m_node(nodeHandle),
      m_maxVelocity(nodeHandle.param<double>("maxVelocity", DEFAULT_MAX_VEL) *
                    METERS_TO_MILLIS),
      m_previousSpeedData(), m_usePings(useTimeouts),
      m_lastIncomingPingTime(std::chrono::steady_clock::now()),
      m_publisher(nodeHandle), m_serial(commsObj), m_services(nodeHandle) {}

/**
 * Runs the main Argo node loop providing communications to and from
 * ROS to the vehicle. It will continue to run until either a shutdown
 * signal is sent by ROS or an exit condition is encountered
 *
 * @param (Unused)
 */
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

/*
 * Sets up the main loop to run, this needs to be called after the
 * object is constructed to start the driver.
 */
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

/*
 * If the driver has timeoutEnabled (see constructor) : Sends a ping to
 * the vehicle and checks a response has been received in time. If
 * timeoutEnabled was false always returns true
 *
 * @return True if a ping has been received before timeout or if
 * timeoutEnabled is set to false. False if a ping has not been received in
 * time.
 */
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

/*
 * Takes the current command string and the parsed type of command and
 * takes appropriate action depending on the command type.
 *
 * @param type The type of command s refers to. See CommandType.
 * @param s The string representation of the command
 */
void ArgoDriver::parseCommand(CommandType type, const std::string &s) {
  switch (type) {
  case CommandType::None:
    return;
  case CommandType::Encoder: {
    auto encoderData = CommsParser::parseEncoderCommand(s);
    m_publisher.publishCurrentOdometry(encoderData);
    m_publisher.publishEncoderCount(encoderData);
    break;
  }
  case CommandType::Fatal: {
    const std::string out{"Got fatal warning from Arduino:\n" + s};
    ROS_FATAL(out.c_str());
    ros::shutdown();
    break;
  }
  case CommandType::Ping: {
    ROS_DEBUG("Received ping");
    m_lastIncomingPingTime = std::chrono::steady_clock::now();
    break;
  }
  case CommandType::Speed: {
    auto speedData = CommsParser::parseSpeedCommand(s);
    m_publisher.publishCurrentSpeed(speedData);
    break;
  }
  case CommandType::Warning: {
    const std::string out{"Got warning from Arduino:\n" + s};
    ROS_WARN(out.c_str());
    break;
  }
  } // End of switch
}

/**
 * Reads from the Arduino and loops over all messages
 * to determine their type (see CommandType) and then
 * dispatch them onto a handler
 */
void ArgoDriver::readFromArduino() {
  // Read from Arduino
  const auto serialInput = m_serial.read();
  for (const auto &incomingString : serialInput) {
    auto commandType = CommsParser::parseIncomingBuffer(incomingString);
    parseCommand(commandType, incomingString);
  }
}

/*
 * Updates the target speed of the vehicle if the target has changed
 * by appending a command to the output buffer of this loop. If
 * the requested speed was greater than the maximum velocity it
 * constrains this speed to the maximum velocity and emits a ROS
 * warning.
 */
void ArgoDriver::updateTargetSpeed() {
  // Get our current target speed and send an update if required
  auto currentSpeedTarget = m_services.getTargetSpeed();
  if (currentSpeedTarget == m_previousSpeedData) {
    return;
  }

  bool speedWasConstrained = false;

  // Constrain to maximum
  if (currentSpeedTarget.leftWheel > m_maxVelocity) {
    const std::string out{
        "Left wheel speed target was: " +
        std::to_string(currentSpeedTarget.leftWheel) +
        "\nRestricting to max speed: " + std::to_string(m_maxVelocity)};
    ROS_WARN(out.c_str());
    currentSpeedTarget.leftWheel = m_maxVelocity;
    speedWasConstrained = true;
  }

  if (currentSpeedTarget.rightWheel > m_maxVelocity) {
    const std::string out{
        "Right wheel speed target was: " +
        std::to_string(currentSpeedTarget.rightWheel) +
        "\nRestricting to max speed: " + std::to_string(m_maxVelocity)};
    ROS_WARN(out.c_str());
    currentSpeedTarget.rightWheel = m_maxVelocity;
    speedWasConstrained = true;
  }

  if (speedWasConstrained) {
    m_services.setTargetSpeed(currentSpeedTarget);
  }

  m_previousSpeedData = currentSpeedTarget;
  m_outputBuffer.push_back(CommsParser::getSpeedCommand(currentSpeedTarget));
}
