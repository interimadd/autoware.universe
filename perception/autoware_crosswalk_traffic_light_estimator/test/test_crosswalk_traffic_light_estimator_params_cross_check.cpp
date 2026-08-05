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
// §10): the same production crosswalk_traffic_light_estimator.param.yaml must build the same
// CrosswalkTrafficLightEstimatorConfig whether read through (a) an rclcpp::Node -- production's
// path -- or (b) a harness ParameterLoader with no rclcpp::Node -- the Component Test's path.
// This package has no declare_*_config() equivalent, so side (a) is a bare rclcpp::Node with
// automatically_declare_parameters_from_overrides(true): every parameter_override -- the yaml,
// loaded via --params-file -- is auto-declared, so get_parameter() alone reproduces what
// CrosswalkTrafficLightEstimatorNode's own declare_parameter<T>() calls would read back.
//

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/component_test_framework/parameter_loader.hpp>
#include <autoware/crosswalk_traffic_light_estimator/crosswalk_traffic_light_estimator.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <string>

namespace
{
using autoware::component_test_framework::ParameterLoader;
using autoware::crosswalk_traffic_light_estimator::CrosswalkTrafficLightEstimatorConfig;

// The ParameterLoader-side mirror of CrosswalkTrafficLightEstimatorNode's declare_parameter<T>()
// calls (node.cpp).
CrosswalkTrafficLightEstimatorConfig build_estimator_config_via_parameter_loader(
  const ParameterLoader & loader)
{
  CrosswalkTrafficLightEstimatorConfig config;
  config.use_last_detect_color = loader.get<bool>("use_last_detect_color");
  config.use_pedestrian_signal_detect = loader.get<bool>("use_pedestrian_signal_detect");
  config.last_detect_color_hold_time = loader.get<double>("last_detect_color_hold_time");
  config.flashing_detection.last_colors_hold_time = loader.get<double>("last_colors_hold_time");
  return config;
}

}  // namespace

TEST(
  CrosswalkTrafficLightEstimatorParamsCrossCheckTest, ParameterLoaderMatchesNodeDeclaredParameters)
{
  // Arrange
  const std::string yaml_path =
    ament_index_cpp::get_package_share_directory("autoware_crosswalk_traffic_light_estimator") +
    "/config/crosswalk_traffic_light_estimator.param.yaml";

  // Act (a): production's path -- see file comment for why a bare auto-declaring rclcpp::Node
  // stands in for CrosswalkTrafficLightEstimatorNode here.
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  node_options.arguments({"--ros-args", "--params-file", yaml_path});
  auto node = std::make_shared<rclcpp::Node>(
    "crosswalk_traffic_light_estimator_params_cross_check", node_options);

  CrosswalkTrafficLightEstimatorConfig config_from_node;
  config_from_node.use_last_detect_color = node->get_parameter("use_last_detect_color").as_bool();
  config_from_node.use_pedestrian_signal_detect =
    node->get_parameter("use_pedestrian_signal_detect").as_bool();
  config_from_node.last_detect_color_hold_time =
    node->get_parameter("last_detect_color_hold_time").as_double();
  config_from_node.flashing_detection.last_colors_hold_time =
    node->get_parameter("last_colors_hold_time").as_double();

  // Act (b): the Component Test's path -- ParameterLoader, no rclcpp::Node.
  ParameterLoader loader;
  loader.merge_yaml_file(yaml_path);
  const CrosswalkTrafficLightEstimatorConfig config_from_loader =
    build_estimator_config_via_parameter_loader(loader);

  // Assert: both paths build the identical config struct from the identical yaml. Also exercises
  // the config by actually constructing an estimator from it, so a future field this test forgets
  // to update fails loudly rather than silently comparing dead fields.
  EXPECT_EQ(config_from_node.use_last_detect_color, config_from_loader.use_last_detect_color);
  EXPECT_EQ(
    config_from_node.use_pedestrian_signal_detect, config_from_loader.use_pedestrian_signal_detect);
  EXPECT_DOUBLE_EQ(
    config_from_node.last_detect_color_hold_time, config_from_loader.last_detect_color_hold_time);
  EXPECT_DOUBLE_EQ(
    config_from_node.flashing_detection.last_colors_hold_time,
    config_from_loader.flashing_detection.last_colors_hold_time);

  EXPECT_NO_THROW({
    autoware::crosswalk_traffic_light_estimator::CrosswalkTrafficLightEstimator estimator(
      config_from_loader);
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
