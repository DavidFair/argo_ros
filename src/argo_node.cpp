#include "ros/ros.h"
#include <memory>
#include <termios.h>

#include "ArgoDriver.hpp"
#include "SerialComms.hpp"

namespace {
// Place globals into an anonymous namespace so they are local to the
// current translation unit
SerialComms m_commsObj;

std::unique_ptr<ros::NodeHandle> m_nodeHandle;
std::unique_ptr<ArgoDriver> m_driver;
} // Anonymous namespace

int main(int argc, char **argv) {
  ros::init(argc, argv, "argo_driver");

  m_nodeHandle = std::make_unique<ros::NodeHandle>("argo_driver");
  m_driver = std::make_unique<ArgoDriver>(m_commsObj, *m_nodeHandle);

  ROS_INFO("Argo Driver - Node started");

  m_driver->setup();

  ROS_INFO("Argo Driver - Setup complete");

  ros::spin();
}