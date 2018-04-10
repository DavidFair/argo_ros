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

#include "ros/ros.h"

#include "SerialComms.hpp"

namespace
{
void getArduinoSerialFlags(termios &serialControl)
{
  // Options taken from https://playground.arduino.cc/Interfacing/LinuxTTY
  serialControl.c_cflag |= CRTSCTS;
  serialControl.c_iflag |= (IGNBRK | BRKINT | IMAXBEL | IXON);
  serialControl.c_oflag |= (ONLCR | OPOST);
  serialControl.c_lflag |= (ISIG | ICANON | IEXTEN | NOFLSH);

  // Turn off echo
  serialControl.c_lflag &= ~(ECHO | ECHOE | ECHOK | ECHONL);
  serialControl.c_iflag &= ~ICRNL;
}

std::string getErrorMsg() { return std::string{strerror(errno)}; }

void throwLinuxError(const std::string &cause)
{
  const std::string e{cause + "\nError was: ' " + getErrorMsg() + "'"};
  ROS_ERROR(e.c_str());
  throw std::runtime_error(e);
}

} // End of anonymous namespace

SerialComms::~SerialComms()
{
  if (!isValidPort)
  {
    return;
  }

  if (::close(fileDescriptor) != 0)
  {
    const std::string e{"Could not close serial port. Error:\n" +
                        getErrorMsg()};
    ROS_ERROR(e.c_str());
  }
}

std::string SerialComms::read()
{
  isSerialValid();

  const int BUF_SIZE = 256;
  char buf[BUF_SIZE];
  memset(buf, 0, BUF_SIZE);
  int count = ::read(fileDescriptor, buf, BUF_SIZE);

  if (count >= 0)
  {
    return std::string(buf);
  }

  if (errno != EAGAIN)
  {
    // Something went wrong
    throwLinuxError("Failed to read from serial");
  }

  // There was no data available in the buffer
  return std::string{};
}

void SerialComms::write(const std::string &s)
{
  isSerialValid();
  int count = ::write(fileDescriptor, s.c_str(), s.size());

  if (count < 0)
  {
    throwLinuxError("Failed to write to serial device");
  }
}

void SerialComms::openPort(const std::string &portAddress, const int baudRate)
{
  fileDescriptor = open(portAddress.c_str(), (O_RDWR | O_NOCTTY | O_NDELAY));
  // Ensure that we are not in blocking mode on the syscall side either
  fcntl(fileDescriptor, F_SETFL, FNDELAY);

  if (fileDescriptor < 0)
  {
    // Failed to open port
    const std::string e{"Could not open port at: " + portAddress +
                        "\nIs the Arduino connected and on the correct TTY?"};
    ROS_ERROR(e.c_str());
    throw std::runtime_error(e);
  }

  setSerialPortSettings(fileDescriptor, baudRate);
  isValidPort = true;
}

void SerialComms::setSerialPortSettings(int fileDescriptor, int baudRate)
{
  // -----------------------------------------------
  // Adapted from https://stackoverflow.com/a/6947758

  struct termios serialOptions;
  memset(&serialOptions, 0, sizeof serialOptions);

  // Get existing tty settings
  if (tcgetattr(fileDescriptor, &serialOptions) != 0)
  {
    throwLinuxError("tcgetattr failed whilst opening serial port");
  }

  speed_t targetBaud = B0;

  if (baudRate == 115200)
  {
    targetBaud = B115200;
  }
  else
  {
    // We cannot naively convert so we need a lookup table. Add elements as needed.
    throw std::invalid_argument("Target Baud rate not in lookup table");
  }

  // Set the baud rate
  cfsetospeed(&serialOptions, targetBaud);
  cfsetispeed(&serialOptions, targetBaud);

  getArduinoSerialFlags(serialOptions);
  if (tcsetattr(fileDescriptor, TCSANOW, &serialOptions) != 0)
  {
    throwLinuxError("tcsetatrr failed to set the serial options");
  }
  // ----------------------------------------------
}

// Private methods:
void SerialComms::isSerialValid() const
{
  if (!isValidPort)
  {
    const std::string e{
        "Serial port was not setup before other functions were called"};
    ROS_ERROR(e.c_str());
    throw std::runtime_error(e);
  }
}