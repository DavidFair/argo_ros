#include "ros/ros.h"

#include "Services.hpp"
#include "argo_driver/SetTargetOdom.h"
#include "argo_driver/SetWheelSpeeds.h"

namespace {
const std::string SET_WHEEL_SPEED_SERV_NAME{"set_target_speed"};
const std::string SET_ODOM_SERV_NAME{"set_target_odom"};
} // End of anonymous namespace

Services::Services(ros::NodeHandle &node) {
  m_targetOdomServ =
      node.advertiseService(SET_ODOM_SERV_NAME, &Services::setTargetOdom, this);

  m_wheelSpeedServ = node.advertiseService(SET_WHEEL_SPEED_SERV_NAME,
                                           &Services::setWheelSpeed, this);
}

bool Services::setTargetOdom(argo_driver::SetTargetOdom::Request &req,
                             argo_driver::SetTargetOdom::Response &response) {
  // TODO
  return true;
}

bool Services::setWheelSpeed(argo_driver::SetWheelSpeeds::Request &req,
                             argo_driver::SetWheelSpeeds::Response &response) {
  // TODO
  return true;
}