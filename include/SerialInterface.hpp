#ifndef SERIAL_INTERFACE_HPP_
#define SERIAL_INTERFACE_HPP_

#include <string>
#include <vector>

/// Provides an abstract interface to a serial device
class SerialInterface {
public:
  /// Reads the current buffer from the serial device
  virtual std::vector<std::string> read() = 0;
  /// Writes a given collection of strings to a serial device
  virtual bool write(const std::vector<std::string> &s) = 0;
  /// Opens a port for the serial device
  virtual void openPort(const std::string &portAddress, int baudRate) = 0;

protected:
  SerialInterface() = default;
};

#endif // SERIAL_INTERFACE_HPP_