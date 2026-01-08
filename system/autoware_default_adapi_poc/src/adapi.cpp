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
  std::vector<RelayTopic> relay_topics = {
    {"input_topic_1", "output_topic_1", "std_msgs/msg/String"},
    {"input_topic_2", "output_topic_2", "std_msgs/msg/String"},
    // Add more topics as needed
  };
  for (const auto & relay_topic : relay_topics) {
    relays_.emplace_back(*this, relay_topic);
  }
}

}  // namespace autoware::default_adapi

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::default_adapi::AdapiNode)
