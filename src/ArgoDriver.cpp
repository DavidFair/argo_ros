#include "ros/ros.h"

#include "ArgoDriver.hpp"
#include "CommsParser.hpp"
#include "Publisher.hpp"
#include "SerialInterface.hpp"

namespace {
const std::string DEFAULT_TTY = "/dev/ttyACM0";
const double DEFAULT_MAX_VEL = 3; // Meters per second

const double LOOP_TIMER = 50; // Time in ms per loop
const double MILLIS_PER_METER = 1000;
const double MILLIS_PER_SEC = 1000;

} // Anonymous namespace

ArgoDriver::ArgoDriver(SerialInterface &commsObj, ros::NodeHandle &nodeHandle)
    : m_node(nodeHandle),
      m_maxVelocity(nodeHandle.param<double>("maxVelocity", DEFAULT_MAX_VEL) *
                    MILLIS_PER_METER),
      m_previousSpeedData(), m_publisher(nodeHandle), m_serial(commsObj),
      m_services(nodeHandle) {}

void ArgoDriver::loop(const ros::TimerEvent &event) {
  // Prevent the timer from firing again until we are finished
  m_loopTimer.stop();

  // Read from Arduino
  const std::string serialInput = m_serial.read();
  auto commandType = CommsParser::parseIncomingBuffer(serialInput);
  parseCommand(commandType, serialInput);

  // Get our current target speed and send an update if required
  auto currentSpeedTarget = m_services.getTargetSpeed();
  if (!(currentSpeedTarget == m_previousSpeedData)) {
    m_previousSpeedData = currentSpeedTarget;
    m_serial.write(CommsParser::getSpeedCommand(currentSpeedTarget));
  }

  // Start any pending timers if there are any
  m_services.startTimers();

  m_loopTimer.start();
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

// Private methods:
void ArgoDriver::parseCommand(CommandType type, const std::string &s) {
  switch (type) {
  case CommandType::None:
    return;
  case CommandType::Encoder: {
    auto encoderData = CommsParser::parseEncoderCommand(s);
    m_publisher.publishEncoderCount(encoderData);
    break;
  }
  case CommandType::Speed: {
    auto speedData = CommsParser::parseSpeedCommand(s);
    m_publisher.publishCurrentSpeed(speedData);
    break;
  }
  }
}