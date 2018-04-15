#ifndef ARGO_GLOBALS_HPP_
#define ARGO_GLOBALS_HPP_

/// The length between the wheels on the argo
constexpr double g_LENGTH_BETWEEN_WHEELS{1.473}; // Meters

/// The circumference of each wheel
constexpr double g_WHEEL_CIRC{1.91}; // Meters

/// Encoder counts per whole wheel rotation
constexpr int g_ENC_COUNTS_PER_WHEEL_ROT{490};

constexpr double g_DIST_PER_ENC_COUNT{g_WHEEL_CIRC /
                                      g_ENC_COUNTS_PER_WHEEL_ROT};

constexpr int METERS_TO_MILLIS{1000};
const double MILLIS_PER_SEC = 1000;

#endif