#include <string>
#include <gmock/gmock.h>

#include "SerialInterface.hpp"

class SerialCommsMock : public SerialInterface
{
  public:
    MOCK_METHOD0(read, std::string());
    MOCK_METHOD1(write, void(const std::string &));
    MOCK_METHOD2(openPort, void(const std::string &, int));
};