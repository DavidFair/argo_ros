#include <atomic>
#include <chrono>
#include <cmath>
#include <thread>

#include "ros/ros.h"
#include "gtest/gtest.h"

#include "ArgoGlobals.hpp"
#include "CommsParser.hpp"
#include "Publisher.hpp"
#include "argo_driver/CurrentOdom.h"

namespace {

template <typename msgT> class rosSubscriber {
public:
  rosSubscriber(ros::NodeHandle &node, const std::string &topicName)
      : m_subscriber(node.subscribe(topicName, 1,
                                    &rosSubscriber<msgT>::rosCallback, this)) {}

  void rosCallback(const msgT &) { m_callbackTriggered = true; }
  bool m_callbackTriggered{false};

private:
  ros::Subscriber m_subscriber;
};

} // Anonymous namespace
