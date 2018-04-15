#ifndef SERVICES_HPP_
#define SERVICES_HPP_

#include "ros/ros.h"

#include "CommsParser.hpp"
#include "argo_driver/SetTargetOdom.h"
#include "argo_driver/SetWheelSpeeds.h"
#include "argo_driver/Stop.h"

/// A class implementing ROS services for the driver
class Services {
public:
  /// Constructor that creates the services handles in ROS
  Services(ros::NodeHandle &node);

  /// Gets the most recent target speed. See (SpeedData)
  SpeedData getTargetSpeed() const { return m_currentTargetSpeed; };

  /// Starts a timer for changing speed when turning if there are any
  void startTimers();

private:
  /// Creates a timer for converting a turn to a straight line
  void createStraightLineTimer(const ros::Duration &time,
                               const int targetSpeed);

  /// Sets a new target speed to be held internally
  void setNewSpeed(const int leftWheel, const int rightWheel);

  /// Implements the targetOdom service
  bool setTargetOdom(argo_driver::SetTargetOdom::Request &req,
                     argo_driver::SetTargetOdom::Response &response);

  /// Implements the targetSpeed service
  bool setWheelSpeed(argo_driver::SetWheelSpeeds::Request &req,
                     argo_driver::SetWheelSpeeds::Response &response);

  /// Implements the stopVehicle service
  bool stopVehicle(argo_driver::Stop::Request &req,
                   argo_driver::Stop::Response &response);

  /// Handle to the current ROS node
  ros::NodeHandle m_node;

  /// Most recent speed set internally
  SpeedData m_currentTargetSpeed;
  /// Flag to determine if any timers need to be started
  bool m_timerPending;
  /// Timer to fire when going from turn to straight. (See
  /// createStraightLineTimer())
  ros::Timer m_turnTimer;

  /// Handle to target odometery service
  ros::ServiceServer m_targetOdomServ;
  /// Handle to stop vehicle service
  ros::ServiceServer m_stopVehicleServ;
  /// Handle to set wheel speeds service
  ros::ServiceServer m_wheelSpeedServ;
};

#endif // SERVICES_HPP_