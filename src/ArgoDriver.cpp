#include "ros/ros.h"

#include "ArgoDriver.hpp"

namespace {
const double LOOP_TIMER = 100; // Time in ms

const double MILLIS_PER_SEC = 1000;

} // Anonymous namespace

ArgoDriver::ArgoDriver(SerialInterface &commsObj, ros::NodeHandle &nodeHandle)
    : m_node(nodeHandle), m_serial(commsObj) {}

void ArgoDriver::loop(const ros::TimerEvent &event) {
  m_loopTimer.stop();

  m_loopTimer.start();
}

void ArgoDriver::setup() {
  // Setup main loop to fire
  const bool isOneShot = false;
  m_loopTimer = m_node.createTimer(ros::Duration(LOOP_TIMER / 1000),
                                   &ArgoDriver::loop, this, isOneShot);
}