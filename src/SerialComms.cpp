// Serial headers
#include <termios.h>
#include <unistd.h>
// Open headers
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <atomic>
#include <cstring>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "ros/ros.h"

#include "SerialComms.hpp"

namespace {
/*
 * Sets serial flags for on a termios object to communicate with
 * the Arduino. This includes turning echo off, additional EOL
 * processing off and setting rate control options as taken from
 * Arduino documentation.
 *
 * @param serialControl The read termios structure which is modified in place
 */
void getArduinoSerialFlags(termios &serialControl) {
  // Options adapted from
  // https://github.com/todbot/arduino-serial/blob/master/arduino-serial-lib.c
  serialControl.c_cflag &= ~PARENB;
  serialControl.c_cflag &= ~CSTOPB;
  serialControl.c_cflag &= ~CSIZE;
  serialControl.c_cflag |= CS8;
  // no flow control
  serialControl.c_cflag &= ~CRTSCTS;

  serialControl.c_cflag &= ~HUPCL; // disable hang-up-on-close to avoid reset

  serialControl.c_cflag |= CREAD | CLOCAL; // turn on READ & ignore ctrl lines
  serialControl.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl

  // serialControl.c_lflag |= ICANON;                 // Get whole lines only
  serialControl.c_lflag &= ~(ICANON, ECHO | ECHOE | ISIG); // make raw
  serialControl.c_oflag &= ~OPOST;                         // make raw
}

/*
 * Returns a string representation of the internal error message as
 * set by c functions - errno.
 *
 * @return String error message for the last library error
 */
std::string getErrorMsg() { return std::string{strerror(errno)}; }

/*
 * Splits a given string by a given delimeter into a vector of strings
 *
 * @param s The string to split
 * @param delim The delimeter to split the string with
 *
 * @return A vector of all found strings
 */
std::vector<std::string> splitByToken(const std::string &s, const char delim) {
  std::vector<std::string> foundLines;
  std::string foundLine;
  std::stringstream inputStream(s);

  while (std::getline(inputStream, foundLine, delim)) {
    foundLines.push_back(foundLine);
  }

  return foundLines;
}

/*
 * Throws a runtime error and prints a ROS error containing the current
 * library error text and a given explanation from the passed parameter.
 *
 * @param cause The preceeding error text explaining the cause of this error.
 * @throw std::runtime_error Containing the passed and library error texts
 */
void throwLinuxError(const std::string &cause) {
  const std::string e{cause + "\nError was: ' " + getErrorMsg() + "'"};
  ROS_ERROR(e.c_str());
  throw std::runtime_error(e);
}

} // End of anonymous namespace

/*
 * Custom destructor which attempts to close the file descriptor
 * on object destruction. If the resource cannot be close a ROS
 * error is printed.
 */
SerialComms::~SerialComms() {
  // Kill the currently running thread
  m_keepRunning = false;
  m_readThread.join();

  if (!isValidPort) {
    return;
  }

  if (::close(fileDescriptor) != 0) {
    const std::string e{"Could not close serial port. Error:\n" +
                        getErrorMsg()};
    ROS_ERROR(e.c_str());
  }
}

/*
 * Reads from any pending internal commands from an internal buffer.
 * These strings are split by their EOL character and returned as a vector
 * of strings.
 * Only canonical strings (i.e. complete to EOL) are returned. The
 * internal buffer is also cleared.
 *
 * This is non-blocking.
 *
 * @return A vector of complete strings received from the device
 */
std::vector<std::string> SerialComms::read() {
  m_readBufferMutex.lock();
  auto lastNewline = m_pendingReadBuffer.find_last_of('\n');
  m_readBufferMutex.unlock();

  if (lastNewline == std::string::npos) {
    // No newline char to send
    return {};
  }

  m_readBufferMutex.lock();
  const auto parsedString = m_pendingReadBuffer.substr(0, lastNewline);
  m_pendingReadBuffer = m_pendingReadBuffer.substr(lastNewline + 1);
  m_pendingReadBuffer.clear();
  m_readBufferMutex.unlock();

  // Split if there are multiple commands
  const char EOLToken = '\n';
  const auto foundStrings = splitByToken(parsedString, EOLToken);

  for (const auto &inString : foundStrings) {
    const std::string out{"Read in: " + inString};
    ROS_DEBUG(out.c_str());
  }

  return foundStrings;
}

/*
 * Writes the given vector of strings to the output buffer. It is
 * expected all strings already have their EOL character written.
 * If the write fails as the outgoing buffer is full it returns false, if
 * multiple commands were passed it is unspecified how many were succesfully
 * sent.
 * If the write fails for any other reason an exception is thrown with the
 * reason
 * If the write succeeds true is returned.
 *
 * @param string A vector of strings to write to the device
 *
 * @return True if all strings were written, false if the outgoing buffer was
 * full
 * @throws std::runtime_error If the write fails for any other reason
 */
bool SerialComms::write(const std::vector<std::string> &strings) {
  isSerialValid();

  std::string toOutput;

  for (const auto &i : strings) {
    toOutput.append(i);
  }

  const std::string out{"Writing to Arduino:\n" + toOutput};
  ROS_DEBUG(out.c_str());

  int count = ::write(fileDescriptor, toOutput.c_str(), toOutput.size());

  if (count >= 0) {
    // Write was good
    return true;
  }

  if (errno == EAGAIN) {
    ROS_WARN("Could not write the serial port. The output buffer is full.");
  } else {
    throwLinuxError("Failed to write to serial device");
  }
  return false;
}

/*
 * Attempts to open a port at the given address and baud rate.
 *
 * If the device cannot be opened an exception is thrown with an
 * error message describing the reason.
 *
 * This function must succeed before read or write can be called on the
 * object.
 *
 * @param portAddress The address of the device to use. For example
 * /dev/ttyACM0
 * @param baudRate The baud rate to communicate with
 *
 * @throws std::runtime_error If the port cannot be opened with the reason why
 */
void SerialComms::openPort(const std::string &portAddress, const int baudRate) {
  fileDescriptor = open(portAddress.c_str(), (O_RDWR | O_NOCTTY));
  // Sets the file descriptor to blocking mode
  fcntl(fileDescriptor, F_SETFL, 0);

  if (fileDescriptor < 0) {
    // Failed to open port
    const std::string e{
        "Could not open port at: " + portAddress +
        "\nIs the Arduino connected, on the correct TTY and unused?"};
    ROS_ERROR(e.c_str());
    throw std::runtime_error(e);
  }

  setSerialPortSettings(fileDescriptor, baudRate);
  isValidPort = true;

  // Start a loop to read from the serial port
  m_readThread = std::thread(&SerialComms::readToInternalBuffer, this);
}

/**
 * Sets port settings on the passed, opened file descriptor. These
 * settings include blocking mode, raw mode, and various
 * serial terminal specific settings.
 * Throws an exception if the terminal attributes cannot be retrieved
 * or set.
 *
 * The baud rate is used in a lookup table so each new rate must be
 * implemented before it can be used. Current implemented rates are:
 * - 115200 -
 *
 * @param fileDescriptor An opened file descriptor to set properties on
 * @param baudRate The target baud rate to operate at.
 *
 * @throws std::runtime_error If attributes cannot be set or got from the dev
 */
void SerialComms::setSerialPortSettings(int fileDescriptor, int baudRate) {
  // -----------------------------------------------
  // Adapted from https://stackoverflow.com/a/6947758

  struct termios serialOptions;
  memset(&serialOptions, 0, sizeof serialOptions);

  // Get existing tty settings
  if (tcgetattr(fileDescriptor, &serialOptions) != 0) {
    throwLinuxError("tcgetattr failed whilst opening serial port");
  }

  speed_t targetBaud = B0;

  if (baudRate == 115200) {
    targetBaud = B115200;
  } else {
    // We cannot naively convert so we need a lookup table. Add elements as
    // needed.
    throw std::invalid_argument("Target Baud rate not in lookup table");
  }

  // Set the baud rate
  cfsetospeed(&serialOptions, targetBaud);
  cfsetispeed(&serialOptions, targetBaud);

  getArduinoSerialFlags(serialOptions);
  if (tcsetattr(fileDescriptor, TCSANOW, &serialOptions) != 0) {
    throwLinuxError("tcsetatrr failed to set the serial options");
  }
  // ----------------------------------------------
}

// Private methods:
/**
 * Reads any incoming command from the OS buffer to an internal buffer
 * This is primarily used in read() and write() to avoid deadlocks
 * where the buffer cannot write until its contents are read from
 * the device.
 *
 * This function is intended to be called in a thread and will
 * lock and unlock the mutex
 *
 *
 * @throws std::runtime_error If any other error than EAGAIN
 * (meaning buffer empty) is returned with that error in the exception.
 */
void SerialComms::readToInternalBuffer() {
  isSerialValid();
  const int BUF_SIZE = 256;

  while (m_keepRunning) {
    char buf[BUF_SIZE];
    memset(buf, 0, BUF_SIZE);
    int count = ::read(fileDescriptor, buf, BUF_SIZE);

    if (count > 0) {
      m_readBufferMutex.lock();
      m_pendingReadBuffer.append((buf));
      m_readBufferMutex.unlock();
    } else if (count < 0) {
      if (errno != EAGAIN) {
        // Something went wrong
        throwLinuxError("Failed to read from serial");
      }
    }

  } // End of loop
}

/**
 * Checks whether the internal file descriptor has been setup and is valid.
 * Prints a ROS error and throws an exception if the port is not valid.
 *
 * @throws std::runtime_error If the serial port was not setup.
 */
void SerialComms::isSerialValid() const {
  if (!isValidPort) {
    const std::string e{
        "Serial port was not setup before other functions were called"};
    ROS_ERROR(e.c_str());
    throw std::runtime_error(e);
  }
}