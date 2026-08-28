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
// Unit test for declare_fusion_config(): reads the package's own default config yaml through a
// real rclcpp::Node and checks that every value lands in the field of TrafficLightFusionConfig
// that the corresponding production Node's parameter feeds today. No GPU and no model files are
// involved -- none of the three back-end cores touch TensorRT -- and declare_fusion_config()
// never constructs a core, so this test is cheap and always runs.
//
// The point of the test is the three prefixes (multi_camera_fusion. / arbiter. /
// crosswalk_estimator.): a bug that read the wrong subtree would silently give one component
// another component's value.
//

#include "../../src/traffic_light_fusion/traffic_light_fusion_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

namespace
{
namespace tl = autoware::traffic_light;

std::string default_config_path()
{
  return ament_index_cpp::get_package_share_directory("autoware_traffic_light_pipeline") +
         "/config/traffic_light_fusion.param.yaml";
}

// Builds a node with the package's default config yaml, plus any extra `-p name:=value`
// overrides the individual test needs.
std::shared_ptr<rclcpp::Node> make_test_node(
  const std::string & name, const std::vector<std::string> & overrides = {})
{
  std::vector<std::string> args{"--ros-args", "--params-file", default_config_path()};
  for (const auto & override : overrides) {
    args.push_back("-p");
    args.push_back(override);
  }
  rclcpp::NodeOptions options;
  options.arguments(args);
  return std::make_shared<rclcpp::Node>(name, options);
}

TEST(FusionParamsTest, ConfigMatchesDefaultYaml)
{
  // Arrange
  auto node = make_test_node("test_traffic_light_fusion_params");

  // Act
  const auto config = tl::declare_fusion_config(node.get());

  // Assert -- multi_camera_fusion.*
  EXPECT_DOUBLE_EQ(config.multi_camera_fusion.message_lifespan, 0.09);
  EXPECT_DOUBLE_EQ(config.multi_camera_fusion.prior_log_odds, 0.0);
  EXPECT_FALSE(config.multi_camera_fusion.use_signal_consistency_check);
  EXPECT_FALSE(config.multi_camera_fusion.publish_partial_matched_signal);
  // The map is a construction-time input to TrafficLightFusion, never a parameter.
  EXPECT_EQ(config.multi_camera_fusion.lanelet_map_ptr, nullptr);

  // Assert -- arbiter.*
  EXPECT_DOUBLE_EQ(config.arbiter.external_delay_tolerance, 5.0);
  EXPECT_DOUBLE_EQ(config.arbiter.external_time_tolerance, 5.0);
  EXPECT_DOUBLE_EQ(config.arbiter.perception_time_tolerance, 1.0);
  EXPECT_EQ(config.arbiter.source_priority, "confidence");
  EXPECT_FALSE(config.arbiter.enable_signal_matching);

  // Assert -- crosswalk_estimator.*
  EXPECT_TRUE(config.crosswalk_estimator.use_last_detect_color);
  EXPECT_TRUE(config.crosswalk_estimator.use_pedestrian_signal_detect);
  EXPECT_DOUBLE_EQ(config.crosswalk_estimator.last_detect_color_hold_time, 2.0);
  EXPECT_DOUBLE_EQ(config.crosswalk_estimator.flashing_detection.last_colors_hold_time, 1.0);
}

// Each prefix must be read from its own subtree: this pins the values apart so a copy-paste bug
// between the three prefixes shows up as a wrong value rather than a coincidental match.
TEST(FusionParamsTest, EachPrefixIsReadFromItsOwnSubtree)
{
  // Arrange
  auto node = make_test_node(
    "test_traffic_light_fusion_params_prefixes",
    {"multi_camera_fusion.message_lifespan:=0.11", "arbiter.perception_time_tolerance:=2.5",
     "crosswalk_estimator.last_detect_color_hold_time:=3.5",
     "crosswalk_estimator.last_colors_hold_time:=4.5"});

  // Act
  const auto config = tl::declare_fusion_config(node.get());

  // Assert
  EXPECT_DOUBLE_EQ(config.multi_camera_fusion.message_lifespan, 0.11);
  EXPECT_DOUBLE_EQ(config.arbiter.perception_time_tolerance, 2.5);
  EXPECT_DOUBLE_EQ(config.crosswalk_estimator.last_detect_color_hold_time, 3.5);
  EXPECT_DOUBLE_EQ(config.crosswalk_estimator.flashing_detection.last_colors_hold_time, 4.5);
}

// TrafficLightArbiter treats every unrecognized source_priority as "confidence" silently;
// declare_fusion_config() normalizes it (and warns) instead, exactly as TrafficLightArbiterNode
// does.
TEST(FusionParamsTest, UnknownSourcePriorityFallsBackToConfidence)
{
  // Arrange
  auto node = make_test_node(
    "test_traffic_light_fusion_params_source_priority", {"arbiter.source_priority:=typo"});

  // Act
  const auto config = tl::declare_fusion_config(node.get());

  // Assert
  EXPECT_EQ(config.arbiter.source_priority, "confidence");
}

TEST(FusionParamsTest, KnownSourcePriorityIsPreserved)
{
  // Arrange
  auto node = make_test_node(
    "test_traffic_light_fusion_params_source_priority_external",
    {"arbiter.source_priority:=external"});

  // Act
  const auto config = tl::declare_fusion_config(node.get());

  // Assert
  EXPECT_EQ(config.arbiter.source_priority, "external");
}
}  // namespace

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
