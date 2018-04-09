#include "ros/ros.h"
#include <termios.h>

#include "SerialComms.hpp"

SerialComms commsObj;

int main(int argc, char **argv) {
  ros::init(argc, argv, "argo_driver");

  ros::NodeHandle nodeHandle;

  const double NODE_FREQ = 10; // 10Hz
  ros::Rate loopTimer(NODE_FREQ);
}