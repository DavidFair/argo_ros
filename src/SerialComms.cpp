// Serial headers
#include <termios.h>
#include <unistd.h>
// Open headers
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <cstring>
#include <stdexcept>
#include <string>

#include "SerialComms.hpp"

namespace {
void getArduinoSerialFlags(termios &serialControl) {
  // Options taken from https://playground.arduino.cc/Interfacing/LinuxTTY
  serialControl.c_cflag |= CRTSCTS;
  serialControl.c_iflag |= (IGNBRK | BRKINT | ICRNL | IMAXBEL | IXON);
  serialControl.c_oflag |= (ONLCR | OPOST);
  serialControl.c_lflag |= (ISIG | ICANON | IEXTEN | NOFLSH);
  // Turn off echo
  serialControl.c_lflag &= ~(ECHO | ECHOE | ECHOK | ECHONL);
}
} // End of anonymous namespace

std::string SerialComms::read() { return std::string{}; }

void SerialComms::write(const std::string &s) {}

void SerialComms::openPort(const std::string &portAddress, const int baudRate) {
  fileDescriptor = open(portAddress.c_str(), (O_RDWR | O_NOCTTY | O_NDELAY));
  if (fileDescriptor < 0) {
    // Failed to open port
    const std::string e{"Could not open port at: " + portAddress};
    throw std::runtime_error(e);
  }

  setSerialPortSettings(fileDescriptor, baudRate);
}

void SerialComms::setSerialPortSettings(int fileDescriptor, int baudRate) {
  // -----------------------------------------------
  // Adapted from https://stackoverflow.com/a/6947758

  struct termios serialOptions;
  memset(&serialOptions, 0, sizeof serialOptions);

  // Get existing tty settings
  if (tcgetattr(fileDescriptor, &serialOptions) != 0) {
    const std::string e = "error " + std::to_string(errno) + " from tcgetattr";
    throw std::runtime_error(e);
  }

  // Set the baud rate
  cfsetospeed(&serialOptions, static_cast<speed_t>(baudRate));
  cfsetispeed(&serialOptions, static_cast<speed_t>(baudRate));

  getArduinoSerialFlags(serialOptions);
  if (tcsetattr(fileDescriptor, TCSANOW, &serialOptions) != 0) {
    const std::string e = "error " + std::to_string(errno) + " from tcsetattr";
    throw std::runtime_error(e);
  }
  // ----------------------------------------------
}