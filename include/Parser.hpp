#ifndef PARSER_HPP_
#define PARSER_HPP_

#include <string>
#include <vector>

enum class CommandType { None, Encoder, Speed };

struct EncoderData {
  EncoderData() : isValid(false), leftWheel(0), rightWheel(0) {}
  EncoderData(int left, int right)
      : isValid(true), leftWheel(left), rightWheel(right) {}

  bool isValid;
  int leftWheel;
  int rightWheel;
};

struct SpeedData {
  SpeedData() : isValid(false), leftWheel(0), rightWheel(0) {}
  SpeedData(int left, int right)
      : isValid(true), leftWheel(left), rightWheel(right) {}

  bool isValid;
  int leftWheel;
  int rightWheel;
};

class Parser {
  Parser() = delete;

public:
  static CommandType parseIncomingBuffer(const std::string &received);
  static EncoderData parseEncoderCommand(const std::string &input);
  static SpeedData parseSpeedCommand(const std::string &input);

private:
  static CommandType determineCommandType(const std::string &input);

  static std::vector<std::string> splitCommands(const std::string &s);
};

#endif // PARSER_HPP_
