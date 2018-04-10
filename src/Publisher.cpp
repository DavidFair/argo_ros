#include "ros/ros.h"

#include "Publisher.hpp"

namespace
{
// Adapted from https://stackoverflow.com/a/44973498
// ------------------
std::string trimLeading(const std::string &cs)
{
    // Force a copy if input was constant
    std::string s = cs;
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](int c) { return !std::isspace(c); }));
    return s;
}
// ------------------
}

Publisher::Publisher(ros::NodeHandle &handle) : m_handle(handle) {}

void Publisher::parseIncomingBuffer(const std::string &received)
{
    if (received.size() == 0)
    {
        // Nothing was received this loop
        return;
    }

    const std::string trimmedString = trimLeading(received);
    if (trimmedString[0] != '!')
    {
        // Not a command string so print and continue
        const std::string out{"Arduino sent:\n" + received};
        ROS_INFO(out.c_str());
        return;
    }

    determineCommandType(trimmedString);
}

// Private methods

void Publisher::determineCommandType(const std::string &input)
{
    const std::string ENC_PREFIX = "!e";
    const std::string SPEED_PREFIX = "!s";
    if (input.find(ENC_PREFIX) != std::string::npos)
    {
        parseEncoderOutput(input);
    }
    else if (input.find(SPEED_PREFIX) != std::string::npos)
    {
        parseSpeedOutput(input);
    }
    else
    {
        const std::string out{"The following command is unknown:\n" + input};
        ROS_WARN(out.c_str());
    }
}

void Publisher::parseEncoderOutput(const std::string &input)
{
    // TODO
}

void Publisher::parseSpeedOutput(const std::string &input)
{
    // TODO
}