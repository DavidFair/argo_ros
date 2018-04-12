#include <cmath>
#include <math.h>

#include "boost/bind.hpp"
#include "ros/ros.h"

#include "Services.hpp"
#include "argo_driver/SetTargetOdom.h"
#include "argo_driver/SetWheelSpeeds.h"

namespace {
const std::string SET_WHEEL_SPEED_SERV_NAME{"set_target_speed"};
const std::string SET_ODOM_SERV_NAME{"set_target_odom"};
const std::string STOP_VEHICLE_SERV_NAME{"stop_vehicle"};

// The length between the wheels on the argo
const double LENGTH_BETWEEN_WHEELS{1.473}; // Meters

double convertToRadians(double degrees) { return degrees * (M_PI / 180); }
const int METERS_TO_MILLIS{1000};

} // End of anonymous namespace

Services::Services(ros::NodeHandle &node)
    : m_node(node), m_turnTimer(), m_targetOdomServ(), m_stopVehicleServ(),
      m_wheelSpeedServ() {
  m_targetOdomServ =
      node.advertiseService(SET_ODOM_SERV_NAME, &Services::setTargetOdom, this);

  m_wheelSpeedServ = node.advertiseService(SET_WHEEL_SPEED_SERV_NAME,
                                           &Services::setWheelSpeed, this);

  m_stopVehicleServ = node.advertiseService(STOP_VEHICLE_SERV_NAME,
                                            &Services::stopVehicle, this);
}

void Services::setNewSpeed(const int leftWheel, const int rightWheel) {
  SpeedData newTargetSpeed{leftWheel, rightWheel};
  m_currentTargetSpeed = newTargetSpeed;
}

bool Services::setTargetOdom(argo_driver::SetTargetOdom::Request &req,
                             argo_driver::SetTargetOdom::Response &response) {
  // First we need to calculate the time to complete this turn in
  // this is distance to way-point / velocity
  double newSpeed = req.targetSpeed;
  const double time = req.targetDistance / newSpeed;

  // The angular momentum is the change in radians per unit time available
  double deltaTheta = req.isRadians ? req.targetClockwiseRadiansRotation
                                    : req.targetClockwiseDegreesRotation;
  if (!req.isRadians) {
    deltaTheta = convertToRadians(deltaTheta);
  }

  // Normalise to 2PI
  if (deltaTheta > 2 * M_PI) {
    deltaTheta = std::fmod(deltaTheta, (2 * M_PI));
  }

  if (deltaTheta > M_PI_2 && deltaTheta < (M_PI + M_PI_2)) {
    // We are going backwards - subtract Pi radians and reverse speed
    newSpeed = -newSpeed;
    deltaTheta -= M_PI;
  }

  const double angularMomentum = deltaTheta / time;
  // The delta between wheels is given as angularMomentum * Distance between
  // wheels, half of each component is added to each respective wheel
  const double velocityDifference =
      (angularMomentum * LENGTH_BETWEEN_WHEELS) / 2;

  // Convert to millis
  const int velocityDelta = velocityDifference * METERS_TO_MILLIS;
  const int requestedSpeed = newSpeed * METERS_TO_MILLIS;

  const int targetLeftSpeed = requestedSpeed + velocityDelta;
  const int targetRightSpeed = requestedSpeed - velocityDelta;

  setNewSpeed(targetLeftSpeed, targetRightSpeed);

  // Setup a turn timer to fire after the speed has been send when the turn has
  // been completed after time + 10% for PID controller and coms
  bool oneShot = true;
  bool autoStart = false;
  m_turnTimer = m_node.createTimer(
      ros::Duration(time * 1.1),
      boost::bind(&Services::setNewSpeed, this, requestedSpeed, requestedSpeed),
      oneShot, autoStart);
  m_timerPending = true;

  return true;
}

bool Services::setWheelSpeed(argo_driver::SetWheelSpeeds::Request &req,
                             argo_driver::SetWheelSpeeds::Response &response) {
  setNewSpeed(req.leftWheelTarget * METERS_TO_MILLIS,
              req.rightWheelTarget * METERS_TO_MILLIS);
  return true;
}

void Services::startTimers() {
  if (m_timerPending) {
    m_turnTimer.start();
  }
}

bool Services::stopVehicle(argo_driver::Stop::Request &req,
                           argo_driver::Stop::Response &response) {
  SpeedData newTargetSpeed{0, 0};
  m_currentTargetSpeed = newTargetSpeed;
  return true;
}