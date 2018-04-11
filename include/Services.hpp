#ifndef SERVICES_HPP_
#define SERVICES_HPP_

#include "ros/ros.h"

#include "Parser.hpp"
#include "argo_driver/SetTargetOdom.h"
#include "argo_driver/SetWheelSpeeds.h"

class Services {
public:
  Services(ros::NodeHandle &node);

  SpeedData getTargetSpeed() const { return m_currentTargetSpeed; };

private:
  bool setTargetOdom(argo_driver::SetTargetOdom::Request &req,
                     argo_driver::SetTargetOdom::Response &response);

  bool setWheelSpeed(argo_driver::SetWheelSpeeds::Request &req,
                     argo_driver::SetWheelSpeeds::Response &response);

  SpeedData m_currentTargetSpeed;

  ros::ServiceServer m_targetOdomServ;
  ros::ServiceServer m_wheelSpeedServ;
};

#endif // SERVICES_HPP_