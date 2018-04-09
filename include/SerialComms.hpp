#ifndef SERIAL_COMMS_HPP_
#define SERIAL_COMMS_HPP_

#include <string>
#include <termios.h>

#include "SerialInterface.hpp"

class SerialComms : public SerialInterface {
public:
  SerialComms() = default;
  ~SerialComms() = default;

  void openPort(const std::string &portAddress, const int baudRate) override;

  std::string read() override;
  void write(const std::string &s) override;

private:
  void isSerialValid() const;
  void setSerialPortSettings(int fileDescriptor, int baudRate);

  int fileDescriptor{-1};
  bool isValidPort{false};
};

#endif