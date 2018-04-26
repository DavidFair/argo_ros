#include <memory>
#include <signal.h>
#include <termios.h>

#include "ArgoDriver.hpp"
#include "SerialComms.hpp"
#include "ros/ros.h"

namespace {
// Place globals into an anonymous namespace so they are local to the
// current translation unit
SerialComms m_commsObj;

std::unique_ptr<ros::NodeHandle> m_nodeHandle;
std::unique_ptr<ArgoDriver> m_driver;

sig_atomic_t volatile shutdownNode = 0;

} // Anonymous namespace

void sigIntHandler(int) { shutdownNode = 1; }

int main(int argc, char **argv) {
  signal(SIGINT, sigIntHandler);

  ros::init(argc, argv, "argo_driver", ros::init_options::NoSigintHandler);

  m_nodeHandle = std::make_unique<ros::NodeHandle>("argo_driver");
  m_driver = std::make_unique<ArgoDriver>(m_commsObj, *m_nodeHandle);

  ROS_INFO("Argo Driver - Node started");

  m_driver->setup();

  ROS_INFO("Argo Driver - Setup complete");

  ros::Rate spinRate(10); // Hz
  while (shutdownNode == 0) {
    ros::spinOnce();
    spinRate.sleep();
  }

  ROS_INFO("Node is shutting down");

  // Shutting down
  // Ensure the driver is torn down before any ROS stuff
  m_driver.reset();
  ros::shutdown();
}