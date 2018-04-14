#include <gmock/gmock.h>
#include <string>


#include "SerialInterface.hpp"

class SerialCommsMock : public SerialInterface {
public:
  MOCK_METHOD0(read, std::vector<std::string>());
  MOCK_METHOD1(write, bool(const std::vector<std::string> &));
  MOCK_METHOD2(openPort, void(const std::string &, int));
};