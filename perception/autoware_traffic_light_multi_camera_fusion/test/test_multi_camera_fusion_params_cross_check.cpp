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
// §10, mirroring §4 Q2 of the front-end plan): the same production
// traffic_light_multi_camera_fusion.param.yaml must build the same MultiCameraFusionConfig fields
// whether read through (a) an rclcpp::Node -- production's path -- or (b) a harness
// ParameterLoader with no rclcpp::Node -- the Component Test's path. This package has no
// declare_*_config() equivalent (unlike autoware_traffic_light_classifier), so side (a) is a bare
// rclcpp::Node with automatically_declare_parameters_from_overrides(true): every parameter_override
// -- the yaml, loaded via --params-file -- is auto-declared, so get_parameter() alone reproduces
// what MultiCameraFusionNode's own declare_parameter<T>() calls would read back, without
// duplicating the Node's constructor here.
//

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/component_test_framework/parameter_loader.hpp>
#include <autoware/traffic_light_multi_camera_fusion/multi_camera_fusion.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>

namespace
{
namespace tl = autoware::traffic_light;
using autoware::component_test_framework::ParameterLoader;

// The ParameterLoader-side mirror of MultiCameraFusionNode's declare_parameter<T>() calls
// (traffic_light_multi_camera_fusion_node.cpp). camera_namespaces is launch-only (plan §1.3) and
// approximate_sync selects a sync strategy rather than a config field (plan §4.6); neither belongs
// in MultiCameraFusionConfig, so neither is read here.
tl::MultiCameraFusionConfig build_fusion_config_via_parameter_loader(const ParameterLoader & loader)
{
  tl::MultiCameraFusionConfig config;
  config.message_lifespan = loader.get<double>("message_lifespan");
  config.prior_log_odds = loader.get<double>("prior_log_odds");
  config.use_signal_consistency_check = loader.get<bool>("signal_consistency_check.enable");
  config.publish_partial_matched_signal =
    loader.get<bool>("signal_consistency_check.publish_partial_matched_signal");
  return config;
}

// Mirrors the fusion pipeline harness's rejection of approximate_sync: true (plan §4.6):
// approximate_sync selects ApproximateTime sync, which the harness never implements (exact-stamp
// sync only, plan §4.6), so a scenario asking for it must fail loudly at startup instead of
// silently running exact sync under a name that promises something else.
void validate_approximate_sync(const ParameterLoader & loader)
{
  if (loader.get<bool>("approximate_sync")) {
    throw std::runtime_error(
      "approximate_sync: true is not supported by the Component Test harness "
      "(exact-stamp sync only)");
  }
}

}  // namespace

TEST(MultiCameraFusionParamsCrossCheckTest, ParameterLoaderMatchesNodeDeclaredParameters)
{
  // Arrange
  const std::string yaml_path =
    ament_index_cpp::get_package_share_directory("autoware_traffic_light_multi_camera_fusion") +
    "/config/traffic_light_multi_camera_fusion.param.yaml";

  // Act (a): production's path -- see file comment for why a bare auto-declaring rclcpp::Node
  // stands in for MultiCameraFusionNode here.
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  node_options.arguments({"--ros-args", "--params-file", yaml_path});
  auto node =
    std::make_shared<rclcpp::Node>("multi_camera_fusion_params_cross_check", node_options);

  tl::MultiCameraFusionConfig config_from_node;
  config_from_node.message_lifespan = node->get_parameter("message_lifespan").as_double();
  config_from_node.prior_log_odds = node->get_parameter("prior_log_odds").as_double();
  config_from_node.use_signal_consistency_check =
    node->get_parameter("signal_consistency_check.enable").as_bool();
  config_from_node.publish_partial_matched_signal =
    node->get_parameter("signal_consistency_check.publish_partial_matched_signal").as_bool();

  // Act (b): the Component Test's path -- ParameterLoader, no rclcpp::Node.
  ParameterLoader loader;
  loader.merge_yaml_file(yaml_path);
  const tl::MultiCameraFusionConfig config_from_loader =
    build_fusion_config_via_parameter_loader(loader);

  // Assert: both paths build the identical config fields from the identical yaml.
  EXPECT_DOUBLE_EQ(config_from_node.message_lifespan, config_from_loader.message_lifespan);
  EXPECT_DOUBLE_EQ(config_from_node.prior_log_odds, config_from_loader.prior_log_odds);
  EXPECT_EQ(
    config_from_node.use_signal_consistency_check, config_from_loader.use_signal_consistency_check);
  EXPECT_EQ(
    config_from_node.publish_partial_matched_signal,
    config_from_loader.publish_partial_matched_signal);

  // Assert: approximate_sync is a real key in this yaml but build_fusion_config_via_parameter_
  // loader() never reads it (it is a sync-strategy selector, not a config field, plan §4.6), so it
  // must show up as unused -- confirming the allow-list this scenario needs has no gap.
  const auto unused = loader.unused_parameter_names();
  EXPECT_NE(std::find(unused.begin(), unused.end(), "approximate_sync"), unused.end());
}

TEST(MultiCameraFusionParamsCrossCheckTest, ApproximateSyncTrueIsRejected)
{
  ParameterLoader loader;
  loader.set_override("approximate_sync", rclcpp::ParameterValue(true));

  EXPECT_THROW(validate_approximate_sync(loader), std::runtime_error);
}

TEST(MultiCameraFusionParamsCrossCheckTest, ApproximateSyncFalseIsAccepted)
{
  ParameterLoader loader;
  loader.set_override("approximate_sync", rclcpp::ParameterValue(false));

  EXPECT_NO_THROW(validate_approximate_sync(loader));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
