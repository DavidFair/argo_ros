#ifndef SERIAL_COMMS_HPP_
#define SERIAL_COMMS_HPP_

#include <string>
#include <vector>

#include "SerialInterface.hpp"

class SerialComms : public SerialInterface {
public:
  SerialComms() = default;
  ~SerialComms();

  void openPort(const std::string &portAddress, const int baudRate) override;

  std::vector<std::string> read() override;
  bool write(const std::vector<std::string> &s) override;

private:
  void readToInternalBuffer();
  void isSerialValid() const;
  void setSerialPortSettings(int fileDescriptor, int baudRate);

  std::string m_pendingReadBuffer{};

  int fileDescriptor{-1};
  bool isValidPort{false};
};

#endif