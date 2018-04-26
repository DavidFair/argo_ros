#include <gmock/gmock.h>
#include <string>

#include "CommsParser.hpp"

using ::testing::HasSubstr;

namespace {

uint8_t getChecksumVal(const std::string &s) {
  uint8_t sum = 0;
  for (char c : s) {
    sum += (int)c;
  }
  return sum;
}

std::string appendChecksumToString(std::string input) {
  uint8_t checksumVal = getChecksumVal(input);
  input.append(" chk:").append(std::to_string(checksumVal)).append("\n");
  return input;
}

} // End of anonymous namespace

TEST(CommsParserTest, correctCommandTypeIsDetermined) {
  // None
  EXPECT_EQ(CommsParser::parseIncomingBuffer("Hello world"), CommandType::None);

  // Invalid commands should map to none
  EXPECT_EQ(CommsParser::parseIncomingBuffer("!A This is invalid"),
            CommandType::None);

  // Encoder data should map correctly
  EXPECT_EQ(CommsParser::parseIncomingBuffer("!e Encoder Command"),
            CommandType::Encoder);

  // Speed data should map correctly
  EXPECT_EQ(CommsParser::parseIncomingBuffer("!s Speed Command"),
            CommandType::Speed);
}

TEST(CommsParserTest, encoderIsParsed) {
  const int leftEncCount = 9876;
  const int rightEncCount = 8765;

  std::string input = "!e LEFT:" + std::to_string(leftEncCount) +
                      " RIGHT:" + std::to_string(rightEncCount);

  input = appendChecksumToString(std::move(input));

  auto result = CommsParser::parseEncoderCommand(input);
  EXPECT_TRUE(result.isValid);
  EXPECT_EQ(result.leftWheel, leftEncCount);
  EXPECT_EQ(result.rightWheel, rightEncCount);
}

TEST(CommsParserTest, mangledEncoderInputIsHandled) {
  const std::string inputCutOff{"!e LEFT:1234 RIGHT"};
  const std::string inputMangled{"!e LEFT:FAIL RIGHT:FAIL"};

  auto resultCutOff = CommsParser::parseEncoderCommand(inputCutOff);
  EXPECT_FALSE(resultCutOff.isValid);

  auto resultMangled = CommsParser::parseEncoderCommand(inputMangled);
  EXPECT_FALSE(resultMangled.isValid);
}

TEST(CommsParserTest, speedIsParsed) {
  const int leftSpeed = 1234;
  const int rightSpeed = 2345;

  std::string input = "!s LEFT:" + std::to_string(leftSpeed) +
                      " RIGHT:" + std::to_string(rightSpeed);

  input = appendChecksumToString(std::move(input));

  auto result = CommsParser::parseSpeedCommand(input);
  EXPECT_TRUE(result.isValid);
  EXPECT_EQ(result.leftWheel, leftSpeed);
  EXPECT_EQ(result.rightWheel, rightSpeed);
}

TEST(CommsParserTest, pwmIsParsed) {
  const int leftPwm = 123;
  const int rightPwm = 234;

  std::string input = "!m L_PWM:" + std::to_string(leftPwm) +
                      " R_PWM:" + std::to_string(rightPwm);

  input = appendChecksumToString(std::move(input));

  auto result = CommsParser::parsePwmCommand(input);
  EXPECT_TRUE(result.isValid);
  EXPECT_EQ(result.leftWheel, leftPwm);
  EXPECT_EQ(result.rightWheel, rightPwm);
}

TEST(CommsParserTest, mangledSpeedInputIsHandled) {
  const std::string inputCutOff{"!s LEFT_SPEED:1234 RIGHT_SPEED"};
  const std::string inputMangled{"!s LEFT_SPEED:FAIL RIGHT_SPEED:FAIL"};

  auto resultCutOff = CommsParser::parseSpeedCommand(inputCutOff);
  EXPECT_FALSE(resultCutOff.isValid);

  auto resultMangled = CommsParser::parseSpeedCommand(inputMangled);
  EXPECT_FALSE(resultMangled.isValid);
}

TEST(CommsCommands, getSpeedCommand) {
  const int expectedLeftSpeed = 1123;
  const int expectedRightSpeed = 2345;
  SpeedData targetSpeed{expectedLeftSpeed, expectedRightSpeed};

  const std::string expectedString{
      "!T L_SPEED:" + std::to_string(expectedLeftSpeed) +
      " R_SPEED:" + std::to_string(expectedRightSpeed) + " chk:"};

  auto result = CommsParser::getSpeedCommand(targetSpeed);
  EXPECT_THAT(result, HasSubstr(expectedString));
}

TEST(CommsCommands, speedCommandUsesChecksum) {
  const int expectedLeftSpeed = 1234;
  const int expectedRightSpeed = 2345;
  SpeedData targetSpeed{expectedLeftSpeed, expectedRightSpeed};

  const std::string inputString{
      "!T L_SPEED:" + std::to_string(expectedLeftSpeed) +
      " R_SPEED:" + std::to_string(expectedRightSpeed)};
  uint8_t checksumVal = getChecksumVal(inputString);

  const std::string expectedChecksum = {" chk:" + std::to_string(checksumVal)};

  auto result = CommsParser::getSpeedCommand(targetSpeed);
  EXPECT_THAT(result, HasSubstr(expectedChecksum));
}