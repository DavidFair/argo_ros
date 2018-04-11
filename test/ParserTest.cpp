#include <string>

#include <gtest/gtest.h>

#include "Parser.hpp"

TEST(ParserTest, correctCommandTypeIsDetermined) {
  // None
  EXPECT_EQ(Parser::parseIncomingBuffer("Hello world"), CommandType::None);

  // Invalid commands should map to none
  EXPECT_EQ(Parser::parseIncomingBuffer("!A This is invalid"),
            CommandType::None);

  // Encoder data should map correctly
  EXPECT_EQ(Parser::parseIncomingBuffer("!e Encoder Command"),
            CommandType::Encoder);

  // Speed data should map correctly
  EXPECT_EQ(Parser::parseIncomingBuffer("!s Speed Command"),
            CommandType::Speed);
}

TEST(ParserTest, encoderIsParsed) {
  const int leftEncCount = 9876;
  const int rightEncCount = 8765;

  const std::string input = "!e LEFT:" + std::to_string(leftEncCount) +
                            " RIGHT:" + std::to_string(rightEncCount) + '\n';

  auto result = Parser::parseEncoderCommand(input);
  EXPECT_TRUE(result.isValid);
  EXPECT_EQ(result.leftWheel, leftEncCount);
  EXPECT_EQ(result.rightWheel, rightEncCount);
}

TEST(ParserTest, mangledEncoderInputIsHandled) {
  const std::string inputCutOff{"!e LEFT:1234 RIGHT"};
  const std::string inputMangled{"!e LEFT:FAIL RIGHT:FAIL"};

  auto resultCutOff = Parser::parseEncoderCommand(inputCutOff);
  EXPECT_FALSE(resultCutOff.isValid);

  auto resultMangled = Parser::parseEncoderCommand(inputMangled);
  EXPECT_FALSE(resultMangled.isValid);
}

TEST(ParserTest, speedIsParsed) {
  const int leftSpeed = 1234;
  const int rightSpeed = 2345;

  const std::string input = "!s LEFT:" + std::to_string(leftSpeed) +
                            " RIGHT:" + std::to_string(rightSpeed) + '\n';

  auto result = Parser::parseSpeedCommand(input);
  EXPECT_TRUE(result.isValid);
  EXPECT_EQ(result.leftWheel, leftSpeed);
  EXPECT_EQ(result.rightWheel, rightSpeed);
}

TEST(ParserTest, mangledSpeedInputIsHandled) {
  const std::string inputCutOff{"!s LEFT_SPEED:1234 RIGHT_SPEED"};
  const std::string inputMangled{"!s LEFT_SPEED:FAIL RIGHT_SPEED:FAIL"};

  auto resultCutOff = Parser::parseSpeedCommand(inputCutOff);
  EXPECT_FALSE(resultCutOff.isValid);

  auto resultMangled = Parser::parseSpeedCommand(inputMangled);
  EXPECT_FALSE(resultMangled.isValid);
}
