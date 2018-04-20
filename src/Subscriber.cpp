#include <cmath>
#include <math.h>

#include "argo_driver/Speeds.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "std_msgs/Empty.h"

#include "ArgoDriver.hpp"
#include "ArgoGlobals.hpp"
#include "CommsParser.hpp"
#include "Subscriber.hpp"

namespace {
const std::string SET_WHEEL_SPEEDS_TOPIC_NAME{"cmd_wheel_speeds"};
const std::string SET_TWIST_TOPIC_NAME{"cmd_vel"};

const int MAX_TOPIC_QUEUE = 10;

/*
 * Calculates the change in velocity for each wheel (therefore the
 * overall delta between the wheels is 2 * the returned value) to
 * achieve the requested angular momentum
 *
 * @param angularMomentum
 *
 * @return The velocity difference in millimeters / second to apply to
 * each wheel (both negative and positively depending on rotation direction)
 */
int calcVelocityDelta(double angularMomentum) {
  // The delta between wheels is given as angularMomentum * Distance between
  // wheels, half of each component is added to each respective wheel
  const double velocityDifference =
      (angularMomentum * g_LENGTH_BETWEEN_WHEELS) / 2;

  // Convert to millis
  return velocityDifference * METERS_TO_MILLIS;
}

} // End of anonymous namespace

/*
 * Constructs the Subscriber object and sets up ROS Subscriber
 * that this object provides from the passed node.
 *
 * @param node The ROS node to register the Subscriber with
 */
Subscriber::Subscriber(ros::NodeHandle &node)
    : m_twistServ(node.subscribe(SET_TWIST_TOPIC_NAME, MAX_TOPIC_QUEUE,
                                 &Subscriber::setTwist, this)),
      m_wheelSpeedServ(node.subscribe(SET_WHEEL_SPEEDS_TOPIC_NAME,
                                      MAX_TOPIC_QUEUE,
                                      &Subscriber::setWheelSpeed, this)) {}

// ----- Private methods ----

/**
 * Sets a new speed target for the driver to
 * send on the following loop, both parameters
 * are in millimeters per second
 *
 *
 */
void Subscriber::setNewSpeed(const int leftWheel, const int rightWheel) {
  m_lastSpeedTarget = SpeedData{leftWheel, rightWheel};
}

/**
 * Sets the vehicles wheel speeds for the Argo driver
 * based on the ROS twist messages passed in through
 *  a Subscriber callback.
 *
 * @param msg The twist message to process
 */
void Subscriber::setTwist(const geometry_msgs::Twist::ConstPtr &msg) {
  const auto targetAngularVel = -(msg->angular.z);

  // Convert
  const auto targetForwardVel = msg->linear.x;

  const double targetSpeed = targetForwardVel * METERS_TO_MILLIS;
  const int velocityDifference = calcVelocityDelta(targetAngularVel);

  const int targetLeftSpeed = targetSpeed + velocityDifference;
  const int targetRightSpeed = targetSpeed - velocityDifference;

  setNewSpeed(targetLeftSpeed, targetRightSpeed);
}

/**
 * Sets a new target wheel speed for both the left and right wheels
 *
 * @param req The ROS request containing the speeds for the left and right
 * wheels
 *
 */
void Subscriber::setWheelSpeed(const argo_driver::Speeds::ConstPtr &msg) {
  setNewSpeed(msg->leftWheel * METERS_TO_MILLIS,
              msg->rightWheel * METERS_TO_MILLIS);
}
