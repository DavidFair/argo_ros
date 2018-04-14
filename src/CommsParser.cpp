#include <sstream>
#include <string>

#include "ros/ros.h"

#include "CommsParser.hpp"

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

void rosWarnWrapper(const std::string &s) { ROS_WARN(s.c_str()); }

} // End of anonymous namespace

std::string CommsParser::getDeadmanCommand() {
  // !D is the command to enter deadman mode
  return {"!D\n"};
}

std::string CommsParser::getPingCommand() {
  // !P is the ping command
  return {"!P\n"};
}

std::string CommsParser::getSpeedCommand(const SpeedData &data) {
  // Expected format
  // !T L_SPEED:xxxx R_SPEED:xxxx
  std::ostringstream command;
  command << "!T "
          << "L_SPEED:" << data.leftWheel << ' '
          << "R_SPEED:" << data.rightWheel << '\n';
  return command.str();
}

CommandType CommsParser::parseIncomingBuffer(const std::string &received) {
  if (received.size() == 0) {
    // Nothing was received this loop
    return CommandType::None;
  }

  const std::string trimmedString = trimLeading(received);

  if (trimmedString[0] != '!') {
    // Not a command string so print and continue
    const std::string out{"Arduino sent:\n" + received};
    ROS_INFO(out.c_str());
    return CommandType::None;
  }

  return determineCommandType(trimmedString);
}

// Private methods

CommandType CommsParser::determineCommandType(const std::string &input) {
  const std::string ENC_PREFIX = "!e";
  const std::string FATAL_PREFIX = "!f";
  const std::string SPEED_PREFIX = "!s";
  const std::string PING_PREFIX = "!p";
  const std::string WARN_PREFIX = "!w";

  if (input.find(ENC_PREFIX) != std::string::npos) {
    return CommandType::Encoder;
  } else if (input.find(FATAL_PREFIX) != std::string::npos) {
    return CommandType::Fatal;
  } else if (input.find(PING_PREFIX) != std::string::npos) {
    return CommandType::Ping;
  } else if (input.find(SPEED_PREFIX) != std::string::npos) {
    return CommandType::Speed;
  } else if (input.find(WARN_PREFIX) != std::string::npos) {
    return CommandType::Warning;
  }

  rosWarnWrapper("The following command is unknown:\n" + input);
  return CommandType::None;
}

EncoderData CommsParser::parseEncoderCommand(const std::string &input) {
  auto seperateWords = splitCommands(input);

  // Expected format is 5 parts where space and : are delims
  // !e L_ENC:XXXX R_ENC:XXXX
  const size_t expectedParts = 5;
  if (seperateWords.size() != expectedParts) {
    rosWarnWrapper("Encoder output did not split correclty. Input was:\n" +
                   input);
    return {};
  }

  int leftEncoderCount{0}, rightEncoderCount{0};

  // Position of the left and right vals
  const size_t lCountPos = 2, rCountPos = 4;

  try {
    leftEncoderCount = std::stoi(seperateWords[lCountPos]);
    rightEncoderCount = std::stoi(seperateWords[rCountPos]);
  } catch (std::invalid_argument &e) {
    rosWarnWrapper("Failed to convert encoder val which was:\n" + input +
                   "\nException was: " + e.what());
    return {};
  }

  return {leftEncoderCount, rightEncoderCount};
}

SpeedData CommsParser::parseSpeedCommand(const std::string &input) {
  auto seperateWords = splitCommands(input);

  // Expected format is 5 parts where space and : are delims
  // !s L_SPEED:XXXX R_SPEED:XXXX
  const size_t expectedParts = 5;
  if (seperateWords.size() != expectedParts) {
    rosWarnWrapper("Encoder output did not split correclty. Input was:\n" +
                   input);
    return {};
  }

  int leftWheelSpeed{0}, rightWheelSpeed{0};

  // Position of the left and right vals
  const size_t lCountPos = 2, rCountPos = 4;

  try {
    leftWheelSpeed = std::stoi(seperateWords[lCountPos]);
    rightWheelSpeed = std::stoi(seperateWords[rCountPos]);
  } catch (std::invalid_argument &e) {
    rosWarnWrapper("Failed to convert encoder val which was:\n" + input +
                   "\nException was: " + e.what());
    return {};
  }

  return {leftWheelSpeed, rightWheelSpeed};
}

// Adapted from https://stackoverflow.com/a/7621814
// ----------------
std::vector<std::string> CommsParser::splitCommands(const std::string &s) {
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