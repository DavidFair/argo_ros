#include <sstream>
#include <string>

#include "ros/ros.h"

#include "CommsParser.hpp"

namespace {

const std::string ENC_PREFIX = "!e";
const std::string FATAL_PREFIX = "!f";
const std::string SPEED_PREFIX = "!s";
const std::string PING_PREFIX = "!p";
const std::string PWM_PREFIX = "!m"; // As in pw 'M' or motor target
const std::string WARN_PREFIX = "!w";

// Adapted from https://stackoverflow.com/a/44973498
// ------------------

/**
 * Trims leading whitespace from a given string and
 * returns a trimmed copy
 *
 * @param cs The string the trim
 * @return The copied string with leading whitespace trimmed
 */
std::string trimLeading(const std::string &cs) {
  // Force a copy if input was constant
  std::string s = cs;
  s.erase(s.begin(), std::find_if(s.begin(), s.end(),
                                  [](int c) { return !std::isspace(c); }));
  return s;
}
// ------------------

/*
 * Wraps a ROS warning call to allow string literals to be
 * used within the codebase without having the first create
 * a string. The passed string is printed directly as a ROS
 * warning
 *
 * @param s The string to print as a ROS warning
 */
void rosWarnWrapper(const std::string &s) { ROS_WARN(s.c_str()); }

} // End of anonymous namespace

/*
 * Returns a string containing a deadman command
 * @return A string containing a deadman command
 */
std::string CommsParser::getDeadmanCommand() {
  // !D is the command to enter deadman mode
  return {"!D\n"};
}

/*
 * Returns a string containing a ping command
 * @return A string containing a ping command
 */
std::string CommsParser::getPingCommand() {
  // !P is the ping command
  return {"!P\n"};
}

/* Returns a string containing the current speed target
 * which is constructed from the data passed in. (See SpeedData)
 *
 * @param data A SpeedData object with the target speed
 * @return A formatted string with the targeted speed
 */
std::string CommsParser::getSpeedCommand(const SpeedData &data) {
  // Expected format
  // !T L_SPEED:xxxx R_SPEED:xxxx
  std::ostringstream command;
  command << "!T "
          << "L_SPEED:" << data.leftWheel << ' '
          << "R_SPEED:" << data.rightWheel << '\n';
  return command.str();
}

/*
 * Parses the a single command, determines if it is a command or not.
 * If not a command prints the current string to ROS INFO then returns
 * a command type none. Otherwise determines the strings current command
 * type which is returned.
 *
 * @param received The string to parse and determine the type of
 * @return None if the string was not a command, otherwise the CommandType
 */
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
} // End of anonymous namespace

// Private methods

/*
 * Determines the current strings command type. This function
 * expects that any strings which do not start with a command
 * prefix are not passed. Therefore if an unknown command is known
 * a warning is emitted to ROS with the received string.
 *
 * @param input The string to determine the command type of
 * @return The command type of the given string
 */
CommandType CommsParser::determineCommandType(const std::string &input) {
  if (input.find(ENC_PREFIX) != std::string::npos) {
    return CommandType::Encoder;
  } else if (input.find(FATAL_PREFIX) != std::string::npos) {
    return CommandType::Fatal;
  } else if (input.find(PING_PREFIX) != std::string::npos) {
    return CommandType::Ping;
  } else if (input.find(PWM_PREFIX) != std::string::npos) {
    return CommandType::Pwm;
  } else if (input.find(SPEED_PREFIX) != std::string::npos) {
    return CommandType::Speed;
  } else if (input.find(WARN_PREFIX) != std::string::npos) {
    return CommandType::Warning;
  }

  rosWarnWrapper("The following command is unknown:\n" + input);
  return CommandType::None;
}

/*
 * Parses encoder data received from the vehicle. If the input does
 * not match an expected format emits a warning and returns an
 * object whose attributes are marked invalid (See EncoderData)
 * Otherwise takes the parsed elements and returns their data
 * in a new EncoderData object
 *
 * @param input The string describing encoder data to parse
 * @return An invalid EncoderData object if parsing failed, or a
 * EncoderDataobject with the current encoder positions
 */
EncoderData CommsParser::parseEncoderCommand(const std::string &input) {
  auto seperateWords = splitCommands(input);

  // Expected format is 5 parts where space and : are delims
  // !e L_ENC:XXXX R_ENC:XXXX
  const size_t expectedParts = 5;
  if (seperateWords.size() != expectedParts) {
    rosWarnWrapper("Encoder data input did not split correclty. Input was:\n" +
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

/**
 * Parses PWM data received from the vehicle. If the input does
 * not match an expected format: emits a warning and returns an
 * object whose attributes are marked invalid (See PwmData)
 * Otherwise takes the parsed elements and returns their data
 * in a new PwmData object
 *
 * @param input A string describing speed data to parse
 * @return An invalid PwmData object if parsing failed, or a
 * PwmData object with the current encoder positions
 */
PwmData CommsParser::parsePwmCommand(const std::string &input) {
  auto seperateWords = splitCommands(input);

  // Expected format is 5 pars where space and : are delims
  // !s L_PWM:xxx R_PWM:xxx
  const size_t expectedParts = 5;
  if (seperateWords.size() != expectedParts) {
    rosWarnWrapper("PWM input did not split correclty. Input was:\n" + input);
    return {};
  }

  int leftPwmVal{0}, rightPwmVal{0};

  // Position of the left and right vals
  const size_t lCountPos = 2, rCountPos = 4;

  try {
    leftPwmVal = std::stoi(seperateWords[lCountPos]);
    rightPwmVal = std::stoi(seperateWords[rCountPos]);
  } catch (std::invalid_argument &e) {
    rosWarnWrapper("Failed to convert encoder val which was:\n" + input +
                   "\nException was: " + e.what());
    return {};
  }

  return {leftPwmVal, rightPwmVal};
}

/*
 * Parses speed data received from the vehicle. If the input does
 * not match an expected format: emits a warning and returns an
 * object whose attributes are marked invalid (See SpeedData)
 * Otherwise takes the parsed elements and returns their data
 * in a new SpeedData object
 *
 * @param input A string describing speed data to parse
 * @return An invalid SpeedData object if parsing failed, or a
 * SpeedData object with the current speeds
 */
SpeedData CommsParser::parseSpeedCommand(const std::string &input) {
  auto seperateWords = splitCommands(input);

  // Expected format is 5 parts where space and : are delims
  // !s L_SPEED:XXXX R_SPEED:XXXX
  const size_t expectedParts = 5;
  if (seperateWords.size() != expectedParts) {
    rosWarnWrapper("Speed input did not split correclty. Input was:\n" + input);
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
/**
 * Splits multiple commands given in an input strings by the expected
 * delimeter '\n'. These are returned as a vector of strings representing
 * all found commands.
 *
 * @param s The string to split into multiple command strings
 * @return A vector of strings representing all found command strings
 */
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