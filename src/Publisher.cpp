#include <sstream>

#include "ros/ros.h"

#include "Publisher.hpp"

namespace {
// Adapted from https://stackoverflow.com/a/44973498
// ------------------
std::string trimLeading(const std::string &cs) {
  // Force a copy if input was constant
  std::string s = cs;
  s.erase(s.begin(), std::find_if(s.begin(), s.end(),
                                  [](int c) { return !std::isspace(c); }));
  return s;
}
// ------------------

// Adapted from https://stackoverflow.com/a/7621814
// ----------------
std::vector<std::string> splitCommands(const std::string &s) {
  std::vector<std::string> foundWords;
  std::size_t prev = 0, pos;

  // Looking for either a space char or ':'
  while ((pos = s.find_first_of(" :\n", prev)) != std::string::npos) {
    if (pos > prev)
      foundWords.push_back(s.substr(prev, pos - prev));
    prev = pos + 1;
  }

  // Capture the word to the right of the last delimeter
  if (prev < s.length())
    foundWords.push_back(s.substr(prev, std::string::npos));

  return foundWords;
}
// ---------------
} // namespace

Publisher::Publisher(ros::NodeHandle &handle) : m_handle(handle) {}

void Publisher::parseIncomingBuffer(const std::string &received) {
  if (received.size() == 0) {
    // Nothing was received this loop
    return;
  }

  const std::string trimmedString = trimLeading(received);
  if (trimmedString[0] != '!') {
    // Not a command string so print and continue
    const std::string out{"Arduino sent:\n" + received};
    ROS_INFO(out.c_str());
    return;
  }

  determineCommandType(trimmedString);
}

// Private methods

void Publisher::determineCommandType(const std::string &input) {
  const std::string ENC_PREFIX = "!e";
  const std::string SPEED_PREFIX = "!s";
  if (input.find(ENC_PREFIX) != std::string::npos) {
    parseEncoderOutput(input);
  } else if (input.find(SPEED_PREFIX) != std::string::npos) {
    parseSpeedOutput(input);
  } else {
    const std::string out{"The following command is unknown:\n" + input};
    ROS_WARN(out.c_str());
  }
}

void Publisher::parseEncoderOutput(const std::string &input) {
  auto seperateWords = splitCommands(input);

  // Expected format is 5 parts where space and : are delims
  // !e L_ENC:XXXX R_ENC:XXXX
  const size_t expectedParts = 5;
  if (seperateWords.size() != expectedParts) {
    const std::string out =
        "Encoder output did not split correclty. Input was:\n" + input;
    ROS_WARN(out.c_str());
    return;
  }

  int leftEncoderCount{0}, rightEncoderCount{0};

  // Position of the left and right vals
  const size_t lCountPos = 2, rCountPos = 4;

  try {
    leftEncoderCount = std::stoi(seperateWords[lCountPos]);
    rightEncoderCount = std::stoi(seperateWords[rCountPos]);
  } catch (std::invalid_argument &e) {
    const std::string out = "Failed to convert encoder val which was:\n" +
                            input + "\nException was: " + e.what();
    ROS_WARN(out.c_str());
    return;
  }

  publishEncoderCount(leftEncoderCount, rightEncoderCount);
}

void Publisher::parseSpeedOutput(const std::string &input) {
  // TODO
}

void Publisher::publishEncoderCount(const int leftCount, const int rightCount) {
  // TODO
}