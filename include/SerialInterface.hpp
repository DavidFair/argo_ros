#ifndef SERIAL_INTERFACE_HPP_
#define SERIAL_INTERFACE_HPP_

#include <string>
#include <vector>

class SerialInterface {
public:
  virtual std::vector<std::string> read() = 0;
  virtual bool write(const std::vector<std::string> &s) = 0;
  virtual void openPort(const std::string &portAddress, int baudRate) = 0;

protected:
  SerialInterface() = default;
};

#endif // SERIAL_INTERFACE_HPP_