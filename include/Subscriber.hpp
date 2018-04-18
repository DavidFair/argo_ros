#ifndef SUBSCRIBER_HPP_
#define SUBSCRIBER_HPP_

#include "ros/ros.h"

#include "argo_driver/Speeds.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Empty.h"

// Forward declerations
class ArgoDriver;

/// A class implementing ROS Subscriber for the driver
class Subscriber {
public:
  /// Constructor that creates the Subscriber handles in ROS
  Subscriber(ros::NodeHandle &node, ArgoDriver &driverHandle);

private:
  /// Sets a new target speed on the driver handle
  void setNewSpeed(const int leftWheel, const int rightWheel) const;

  /// Implements the targetOdom Subscriber
  void setTwist(const geometry_msgs::Twist::ConstPtr &msg) const;

  /// Implements the targetSpeed Subscriber
  void setWheelSpeed(const argo_driver::Speeds::ConstPtr &msg) const;

  ArgoDriver &m_driverHandle;

  /// Handle to target handle cmd_vel msgs
  ros::Subscriber m_twistServ;
  /// Handle to set wheel speeds
  ros::Subscriber m_wheelSpeedServ;
};

#endif // SUBSCRIBER_HPP_