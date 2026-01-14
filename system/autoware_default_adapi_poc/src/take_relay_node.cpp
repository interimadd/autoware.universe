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

#include <autoware_utils_rclcpp/polling_subscriber.hpp>
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>

#include <memory>
#include <string>
#include <vector>

using namespace std::chrono_literals;

namespace autoware::default_adapi
{

struct TakeRelay
{
  autoware_utils_rclcpp::InterProcessPollingSubscriber<std_msgs::msg::String> sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;

  TakeRelay(
    rclcpp::Node * node,
    const std::string & input_topic_name,
    const std::string & output_topic_name)
  : sub_(node, input_topic_name),
    pub_(node->create_publisher<std_msgs::msg::String>(output_topic_name, rclcpp::QoS(10)))
  {
  }
};

class TakeRelayNode : public rclcpp::Node
{
public:
  explicit TakeRelayNode(const rclcpp::NodeOptions & options)
  : Node("take_relay_node", options)
  {
    // Declare parameter for number of relay topics
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "Number of relay topics to create";
    descriptor.read_only = false;
    this->declare_parameter("num_relay_topics", 1, descriptor);

    // Get the number of relay topics from parameter
    int num_topics = this->get_parameter("num_relay_topics").as_int();

    // Create relay objects
    for (int i = 1; i <= num_topics; ++i) {
      std::string input_topic = "input_topic_" + std::to_string(i);
      std::string output_topic = "output_topic_" + std::to_string(i);
      
      relays_.emplace_back(std::make_shared<TakeRelay>(this, input_topic, output_topic));
    }

    // Create 10Hz timer
    timer_ = this->create_wall_timer(
      100ms, std::bind(&TakeRelayNode::on_timer, this));
  }

private:
  void on_timer()
  {
    for (auto & relay : relays_) {
      // Take data from polling subscriber
      const auto msg_ptr = relay->sub_.take_data();
      
      // If data is available, publish it
      if (msg_ptr) {
        relay->pub_->publish(*msg_ptr);
      }
    }
  }

  std::vector<std::shared_ptr<TakeRelay>> relays_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace autoware::default_adapi

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<autoware::default_adapi::TakeRelayNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
