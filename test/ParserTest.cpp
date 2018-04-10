#include <string>

#include <gtest/gtest.h>

#include "Parser.hpp"

TEST(ParserTest, encoderIsParsed) {
  const int leftEncCount = 9876;
  const int rightEncCount = 8765;

  const std::string input = "!e LEFT:" + std::to_string(leftEncCount) +
                            " RIGHT:" + std::to_string(rightEncCount) + '\n';

  Parser::parseIncomingBuffer(input);
}

TEST(ParserTest, speedIsParsed) {
  const int leftSpeed = 1234;
  const int rightSpeed = 2345;

  const std::string input = "!s LEFT:" + std::to_string(leftSpeed) +
                            " RIGHT:" + std::to_string(rightSpeed) + '\n';
}
