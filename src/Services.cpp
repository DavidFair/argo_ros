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

int calcVelocityDelta(double targetTime, double deltaTheta) {
  // First we need to calculate the time to complete this turn in
  // this is distance to way-point / velocity
  const double angularMomentum = deltaTheta / targetTime;
  // The delta between wheels is given as angularMomentum * Distance between
  // wheels, half of each component is added to each respective wheel
  const double velocityDifference =
      (angularMomentum * LENGTH_BETWEEN_WHEELS) / 2;

  // Convert to millis
  return velocityDifference * METERS_TO_MILLIS;
}

double calcTimeForTurn(double turnRate, double deltaTheta) {
  double angularMomentum = turnRate / LENGTH_BETWEEN_WHEELS;
  return deltaTheta / angularMomentum;
}

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

void Services::startTimers() {
  if (m_timerPending) {
    m_turnTimer.start();
  }
}

// ----- Private methods ----

void Services::createStraightLineTimer(const ros::Duration &time,
                                       const int targetSpeed) {
  // Setup a turn timer to fire after the speed has been send when the turn has
  // been completed after time + 10% for PID controller and coms
  bool oneShot = true;
  bool autoStart = false;
  m_turnTimer = m_node.createTimer(
      time, boost::bind(&Services::setNewSpeed, this, targetSpeed, targetSpeed),
      oneShot, autoStart);
  m_timerPending = true;
}

void Services::setNewSpeed(const int leftWheel, const int rightWheel) {
  SpeedData newTargetSpeed{leftWheel, rightWheel};
  m_currentTargetSpeed = newTargetSpeed;
}

bool Services::setTargetOdom(argo_driver::SetTargetOdom::Request &req,
                             argo_driver::SetTargetOdom::Response &response) {
  // Constants for turning on the spot

  const double MIN_DIST = 0.5; // Meters before we turn on the spot
  const double MIN_THETA =
      convertToRadians(5);      // Anything over 5 is we will process
  const double MIN_SPEED = 0.3; // m/s where we turn on the spot
  const double TURN_SPEED = 1;  // Meters per second to turn on the spot at

  double newSpeed = req.targetSpeed;
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

  // Check if we are going in a straight line and return early
  if (deltaTheta < MIN_THETA) {
    setNewSpeed(newSpeed * METERS_TO_MILLIS, newSpeed * METERS_TO_MILLIS);
    return true;
  }

  // Determine if were turning on the spot
  double targetDistance = req.targetDistance;
  bool turnOnSpot = (newSpeed < MIN_SPEED || targetDistance < MIN_DIST);

  // If we are not turning on the spot and between 90-270 we are going backwards
  if (!turnOnSpot && (deltaTheta > M_PI_2 && deltaTheta < (M_PI + M_PI_2))) {
    // We are going backwards - subtract Pi radians and reverse speed
    newSpeed = -newSpeed;
    deltaTheta -= M_PI;
  }

  bool isClockwise = (deltaTheta >= 0 && deltaTheta <= M_PI);
  if (!isClockwise) {
    // Normalise delta theta to clockwise for future calculations
    deltaTheta -= (M_PI + M_PI_2);
  }

  int targetLeftSpeed{0};
  int targetRightSpeed{0};
  const double targetSpeed = newSpeed * METERS_TO_MILLIS;
  if (!turnOnSpot) {
    const double targetTime = targetDistance / newSpeed;
    const int velocityDifference = calcVelocityDelta(targetTime, deltaTheta);
    // As we normalised the calculation to clockwise check if we need to negate
    // for anti clockwise rotation
    const int rotationVelocity =
        isClockwise ? velocityDifference : -velocityDifference;

    targetLeftSpeed = targetSpeed + rotationVelocity;
    targetRightSpeed = targetSpeed - rotationVelocity;

    createStraightLineTimer(ros::Duration(targetTime * 1.1), targetSpeed);
  } else {
    const double turnTime = calcTimeForTurn(TURN_SPEED, deltaTheta);

    targetLeftSpeed = isClockwise ? TURN_SPEED : -TURN_SPEED;
    targetRightSpeed = isClockwise ? -TURN_SPEED : TURN_SPEED;

    createStraightLineTimer(ros::Duration(turnTime * 1.1), 0);
  }

  setNewSpeed(targetLeftSpeed, targetRightSpeed);

  return true;
}

bool Services::setWheelSpeed(argo_driver::SetWheelSpeeds::Request &req,
                             argo_driver::SetWheelSpeeds::Response &response) {
  setNewSpeed(req.leftWheelTarget * METERS_TO_MILLIS,
              req.rightWheelTarget * METERS_TO_MILLIS);
  return true;
}

bool Services::stopVehicle(argo_driver::Stop::Request &req,
                           argo_driver::Stop::Response &response) {
  SpeedData newTargetSpeed{0, 0};
  m_currentTargetSpeed = newTargetSpeed;
  return true;
}