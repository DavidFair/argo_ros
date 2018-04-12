#include <string>

#include <gtest/gtest.h>

#include "CommsParser.hpp"

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

  const std::string input = "!e LEFT:" + std::to_string(leftEncCount) +
                            " RIGHT:" + std::to_string(rightEncCount) + '\n';

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

  const std::string input = "!s LEFT:" + std::to_string(leftSpeed) +
                            " RIGHT:" + std::to_string(rightSpeed) + '\n';

  auto result = CommsParser::parseSpeedCommand(input);
  EXPECT_TRUE(result.isValid);
  EXPECT_EQ(result.leftWheel, leftSpeed);
  EXPECT_EQ(result.rightWheel, rightSpeed);
}

TEST(CommsParserTest, mangledSpeedInputIsHandled) {
  const std::string inputCutOff{"!s LEFT_SPEED:1234 RIGHT_SPEED"};
  const std::string inputMangled{"!s LEFT_SPEED:FAIL RIGHT_SPEED:FAIL"};

  auto resultCutOff = CommsParser::parseSpeedCommand(inputCutOff);
  EXPECT_FALSE(resultCutOff.isValid);

  auto resultMangled = CommsParser::parseSpeedCommand(inputMangled);
  EXPECT_FALSE(resultMangled.isValid);
}
