#ifndef SUBSCRIBER_HPP_
#define SUBSCRIBER_HPP_

#include "ros/ros.h"

#include "CommsParser.hpp"
#include "argo_driver/Speeds.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Empty.h"

// Forward declerations
class ArgoDriver;

/// A class implementing ROS Subscriber for the driver
class Subscriber {
public:
  /// Constructor that creates the Subscriber handles in ROS
  Subscriber(ros::NodeHandle &node);

  SpeedData getLastSpeedTarget() const { return m_lastSpeedTarget; }

  void resetLastSpeedTarget() { m_lastSpeedTarget = SpeedData(); }

private:
  /// Sets a new target speed on the driver handle
  void setNewSpeed(const int leftWheel, const int rightWheel);

  /// Implements the targetOdom Subscriber
  void setTwist(const geometry_msgs::Twist::ConstPtr &msg);

  /// Implements the targetSpeed Subscriber
  void setWheelSpeed(const argo_driver::Speeds::ConstPtr &msg);

  /// Handle that holds the last speed target
  SpeedData m_lastSpeedTarget;

  /// Handle to target handle cmd_vel msgs
  ros::Subscriber m_twistServ;
  /// Handle to set wheel speeds
  ros::Subscriber m_wheelSpeedServ;
};

#endif // SUBSCRIBER_HPP_