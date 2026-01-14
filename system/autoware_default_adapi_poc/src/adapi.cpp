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

#include "adapi.hpp"

using namespace std::chrono_literals;

namespace autoware::default_adapi
{

Relay::Relay(
  rclcpp::Node & node,
  const RelayTopic relay_topic)
{
  pub_ = node.create_generic_publisher(
    relay_topic.output_topic_name, relay_topic.topic_type,
    rclcpp::QoS(10));

  sub_ = node.create_generic_subscription(
    relay_topic.input_topic_name, relay_topic.topic_type,
    rclcpp::QoS(10).best_effort(),
    [this](std::shared_ptr<rclcpp::SerializedMessage> msg) {
      pub_->publish(*msg);
    });
}

AdapiNode::AdapiNode(const rclcpp::NodeOptions & options)
: Node("adapi_node", options)
{
  // Declare parameter for number of relay topics
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = "Number of relay topics to create";
  descriptor.read_only = false;
  this->declare_parameter("num_relay_topics", 1, descriptor);

  // Get the number of relay topics from parameter
  int num_topics = this->get_parameter("num_relay_topics").as_int();

  // Create relay topics based on the parameter
  std::vector<RelayTopic> relay_topics;
  for (int i = 1; i <= num_topics; ++i) {
    relay_topics.push_back({
      "input_topic_" + std::to_string(i),
      "output_topic_" + std::to_string(i),
      "std_msgs/msg/String"
    });
  }

  for (const auto & relay_topic : relay_topics) {
    relays_.emplace_back(std::make_shared<Relay>(*this, relay_topic));
  }
}

}  // namespace autoware::default_adapi

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::default_adapi::AdapiNode)
