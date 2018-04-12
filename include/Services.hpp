#ifndef SERVICES_HPP_
#define SERVICES_HPP_

#include "ros/ros.h"

#include "CommsParser.hpp"
#include "argo_driver/SetTargetOdom.h"
#include "argo_driver/SetWheelSpeeds.h"
#include "argo_driver/Stop.h"

class Services {
public:
  Services(ros::NodeHandle &node);

  SpeedData getTargetSpeed() const { return m_currentTargetSpeed; };
  void startTimers();

private:
  void createStraightLineTimer(const ros::Duration &time,
                               const int targetSpeed);

  void setNewSpeed(const int leftWheel, const int rightWheel);

  bool setTargetOdom(argo_driver::SetTargetOdom::Request &req,
                     argo_driver::SetTargetOdom::Response &response);

  bool setWheelSpeed(argo_driver::SetWheelSpeeds::Request &req,
                     argo_driver::SetWheelSpeeds::Response &response);

  bool stopVehicle(argo_driver::Stop::Request &req,
                   argo_driver::Stop::Response &response);

  ros::NodeHandle m_node;

  SpeedData m_currentTargetSpeed;
  bool m_timerPending;
  ros::Timer m_turnTimer;

  ros::ServiceServer m_targetOdomServ;
  ros::ServiceServer m_stopVehicleServ;
  ros::ServiceServer m_wheelSpeedServ;
};

#endif // SERVICES_HPP_