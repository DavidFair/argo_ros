#ifndef SERIAL_COMMS_HPP_
#define SERIAL_COMMS_HPP_

#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "SerialInterface.hpp"

/// Creates a concrete implementation of the SerialInterface to a tty device
class SerialComms : public SerialInterface {
public:
  /// Default constructor for SerialComms
  SerialComms() = default;
  /// Custom destructor for SerialComms
  ~SerialComms();

  /// Attempts to open a port at the given address and baud rate
  void openPort(const std::string &portAddress, const int baudRate) override;

  /// Attempts to read any pending messages from the tty device
  std::vector<std::string> read() override;
  /// Attempts to write passed messages to the tty device
  bool write(const std::vector<std::string> &s) override;

private:
  /// Reads any pending messages to an internal buffer for the next read call
  void readToInternalBuffer();
  /// Returns if the serial port has been created and is valid
  void isSerialValid() const;
  /// Sets various terminal settings on the opened port
  void setSerialPortSettings(int fileDescriptor, int baudRate);

  /// Flag to indicate to other threads when to stop running
  std::atomic<bool> m_keepRunning{true};

  /// Thread to read from serial in a loop
  std::thread m_readThread;

  /// Mutex to protect the read buffer
  std::mutex m_readBufferMutex;

  /// Buffers any incoming messages from the tty device pending. (See read())
  std::string m_pendingReadBuffer{};

  /// The file descriptor for the tty device
  int fileDescriptor{-1};
  /// A flag to indicate whether the file descriptor is valid
  bool isValidPort{false};
};

#endif