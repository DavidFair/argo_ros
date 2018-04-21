#ifndef COMMS_PARSER_HPP_
#define COMMS_PARSER_HPP_

#include <string>
#include <vector>

/// Enum of the various command types that can be received
enum class CommandType { None, Encoder, Fatal, Ping, Pwm, Speed, Warning };

/// Struct holding encoder data for the left and right wheel
struct EncoderData {
  EncoderData() : isValid(false), leftWheel(0), rightWheel(0) {}
  EncoderData(int left, int right)
      : isValid(true), leftWheel(left), rightWheel(right) {}

  bool isValid;
  int leftWheel;
  int rightWheel;
};

struct PwmData {
  PwmData() : isValid(false), leftWheel(0), rightWheel(0) {}
  PwmData(int left, int right)
      : isValid(true), leftWheel(left), rightWheel(right) {}

  bool isValid;
  int leftWheel;
  int rightWheel;
};

/// Struct holding speed data for the left and right wheel
struct SpeedData {
  SpeedData() : isValid(false), leftWheel(0), rightWheel(0) {}
  SpeedData(int left, int right)
      : isValid(true), leftWheel(left), rightWheel(right) {}

  bool isValid;
  int leftWheel;
  int rightWheel;
};

/// SpeedData freestanding comparison operator
inline bool operator==(const SpeedData &a, const SpeedData &b) {
  return a.isValid == b.isValid && a.leftWheel == b.leftWheel &&
         a.rightWheel == b.rightWheel;
}

/// A non-instantiatable class which provides static parsing methods
class CommsParser {
public:
  /// Returns a string containing the deadman command
  static std::string getDeadmanCommand();
  /// Returns a string containing the speed command
  static std::string getSpeedCommand(const SpeedData &data);
  /// Returns a string containing the ping command
  static std::string getPingCommand();

  /// Parses the given string and determines its type. (See CommandType)
  static CommandType parseIncomingBuffer(const std::string &received);
  /// Parses an incoming encoder command to return its data. (See EncoderData)
  static EncoderData parseEncoderCommand(const std::string &input);
  /// Parses an incoming ping command
  static PwmData parsePwmCommand(const std::string &input);
  /// Parses an incoming speed command to return its data. (See SpeedData)
  static SpeedData parseSpeedCommand(const std::string &input);

private:
  /// Deleted constructor to ensure the class is non-instantiatable
  CommsParser() = delete;
  /// Provides the logic to determine the command type. (See CommandType)
  static CommandType determineCommandType(const std::string &input);
  /// Splits an incoming string containing n commands into seperate strings
  static std::vector<std::string> splitCommands(const std::string &s);
};

#endif // COMMS_PARSER_HPP_
