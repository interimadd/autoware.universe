// Copyright 2026 The Autoware Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTOWARE_DEFAULT_ADAPI_POC__ADAPI_HPP_
#define AUTOWARE_DEFAULT_ADAPI_POC__ADAPI_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

namespace autoware::default_adapi
{

struct RelayTopic
{
  std::string input_topic_name;
  std::string output_topic_name;
  std::string topic_type;
};

class Relay
{
public:
  explicit Relay(rclcpp::Node & node, const RelayTopic relay_topic);
private:
  rclcpp::GenericPublisher::SharedPtr pub_;
  rclcpp::GenericSubscription::SharedPtr sub_;
};

class AdapiNode : public rclcpp::Node
{
public:
  explicit AdapiNode(const rclcpp::NodeOptions & options);

private:
  std::vector<std::shared_ptr<Relay>> relays_;
};

}  // namespace autoware::default_adapi

#endif  // AUTOWARE_DEFAULT_ADAPI_POC__ADAPI_HPP_
