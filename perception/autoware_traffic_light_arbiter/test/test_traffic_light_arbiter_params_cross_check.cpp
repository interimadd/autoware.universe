// Copyright 2026 TIER IV, Inc.
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

//
// Cross-check for the Component Test framework's ParameterLoader (fusion component test plan
// §10): the same production traffic_light_arbiter.param.yaml must build TrafficLightArbiter's
// constructor arguments identically whether read through (a) an rclcpp::Node -- production's path
// -- or (b) a harness ParameterLoader with no rclcpp::Node -- the Component Test's path. This
// package has no declare_*_config() equivalent, so side (a) is a bare rclcpp::Node with
// automatically_declare_parameters_from_overrides(true): every parameter_override -- the yaml,
// loaded via --params-file -- is auto-declared, so get_parameter() alone reproduces what
// TrafficLightArbiterNode's own declare_parameter<T>() calls would read back.
//

#include "autoware/traffic_light_arbiter/traffic_light_arbiter.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/component_test_framework/parameter_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <string>

namespace
{
namespace tl = autoware::traffic_light;
using autoware::component_test_framework::ParameterLoader;

struct ArbiterCtorArgs
{
  std::string source_priority;
  bool enable_signal_matching;
  double external_delay_tolerance;
  double external_time_tolerance;
  double perception_time_tolerance;
};

// The ParameterLoader-side mirror of the Node's parameter reads that feed the TrafficLightArbiter
// constructor. Unlike the Node, this doesn't normalize an unrecognized source_priority to
// "confidence" -- TrafficLightArbiter/SignalMatchValidator already treat any value other than
// "external"/"perception" as confidence-based selection, so the raw string round-trips
// identically either way.
ArbiterCtorArgs build_arbiter_args_via_parameter_loader(const ParameterLoader & loader)
{
  ArbiterCtorArgs args;
  args.source_priority = loader.get<std::string>("source_priority");
  args.enable_signal_matching = loader.get<bool>("enable_signal_matching");
  args.external_delay_tolerance = loader.get<double>("external_delay_tolerance");
  args.external_time_tolerance = loader.get<double>("external_time_tolerance");
  args.perception_time_tolerance = loader.get<double>("perception_time_tolerance");
  return args;
}

}  // namespace

TEST(TrafficLightArbiterParamsCrossCheckTest, ParameterLoaderMatchesNodeDeclaredParameters)
{
  // Arrange
  const std::string yaml_path =
    ament_index_cpp::get_package_share_directory("autoware_traffic_light_arbiter") +
    "/config/traffic_light_arbiter.param.yaml";

  // Act (a): production's path -- see file comment for why a bare auto-declaring rclcpp::Node
  // stands in for TrafficLightArbiterNode here.
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  node_options.arguments({"--ros-args", "--params-file", yaml_path});
  auto node =
    std::make_shared<rclcpp::Node>("traffic_light_arbiter_params_cross_check", node_options);

  ArbiterCtorArgs args_from_node;
  args_from_node.source_priority = node->get_parameter("source_priority").as_string();
  args_from_node.enable_signal_matching = node->get_parameter("enable_signal_matching").as_bool();
  args_from_node.external_delay_tolerance =
    node->get_parameter("external_delay_tolerance").as_double();
  args_from_node.external_time_tolerance =
    node->get_parameter("external_time_tolerance").as_double();
  args_from_node.perception_time_tolerance =
    node->get_parameter("perception_time_tolerance").as_double();

  // Act (b): the Component Test's path -- ParameterLoader, no rclcpp::Node.
  ParameterLoader loader;
  loader.merge_yaml_file(yaml_path);
  const ArbiterCtorArgs args_from_loader = build_arbiter_args_via_parameter_loader(loader);

  // Assert: both paths build identical constructor arguments from the identical yaml. Also
  // exercises the arguments by actually constructing a TrafficLightArbiter from each, so a future
  // constructor signature change that this test forgets to update fails loudly rather than
  // silently comparing dead fields.
  EXPECT_EQ(args_from_node.source_priority, args_from_loader.source_priority);
  EXPECT_EQ(args_from_node.enable_signal_matching, args_from_loader.enable_signal_matching);
  EXPECT_DOUBLE_EQ(
    args_from_node.external_delay_tolerance, args_from_loader.external_delay_tolerance);
  EXPECT_DOUBLE_EQ(
    args_from_node.external_time_tolerance, args_from_loader.external_time_tolerance);
  EXPECT_DOUBLE_EQ(
    args_from_node.perception_time_tolerance, args_from_loader.perception_time_tolerance);

  EXPECT_NO_THROW({
    tl::TrafficLightArbiter arbiter(
      args_from_loader.source_priority, args_from_loader.enable_signal_matching,
      args_from_loader.external_delay_tolerance, args_from_loader.external_time_tolerance,
      args_from_loader.perception_time_tolerance);
  });
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
