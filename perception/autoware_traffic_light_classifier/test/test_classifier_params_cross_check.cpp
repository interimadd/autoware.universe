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
// Cross-check for the Component Test framework's ParameterLoader (see
// traffic_light_component_test_plan.md §4 Q2): the same production
// car_traffic_light_classifier.param.yaml must produce the same CNNConfig whether it is read
// through (a) an rclcpp::Node calling declare_cnn_config() -- production's path -- or (b) a
// harness ParameterLoader with no rclcpp::Node -- the Component Test's path. Until the two
// packages share one "parameters -> config" conversion function (plan §4 Q2 "mid-term"), this
// mapping exists twice; this test is what keeps the two copies from drifting apart.
//

#include "../src/classifier_params.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/component_test_framework/parameter_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

namespace
{
namespace tl = autoware::traffic_light;
using autoware::component_test_framework::ParameterLoader;

// A real label file is required: declare_cnn_config() opens it and throws if it can't. The model
// itself is never loaded here -- declare_cnn_config() only declares parameters and reads labels,
// it does not build a TensorRT engine -- so no GPU work happens in this test.
constexpr char kLabelFilename[] = "lamp_labels.txt";

// Resolves a file shipped under autoware_data (see the ansible artifacts role), the same way
// test_cnn_classifier.cpp does. An explicit TLC_TEST_DATA_DIR override takes precedence. Returns
// "" when not found, so the caller can self-skip.
std::string resolve_autoware_data_file(const std::string & filename)
{
  std::vector<std::string> candidate_dirs;
  if (const char * override_dir = std::getenv("TLC_TEST_DATA_DIR")) {
    candidate_dirs.emplace_back(override_dir);
  }
  if (const char * home = std::getenv("HOME")) {
    candidate_dirs.emplace_back(
      std::string(home) + "/autoware_data/ml_models/traffic_light_classifier");
    candidate_dirs.emplace_back(std::string(home) + "/autoware_data/traffic_light_classifier");
  }
  for (const auto & dir : candidate_dirs) {
    const std::string candidate = dir + "/" + filename;
    if (std::filesystem::exists(candidate)) {
      return candidate;
    }
  }
  return "";
}

std::vector<std::string> read_label_lines(const std::string & path)
{
  std::ifstream labels_file(path);
  std::vector<std::string> labels;
  std::string label;
  while (std::getline(labels_file, label)) {
    labels.push_back(label);
  }
  return labels;
}

// The ParameterLoader-side mirror of declare_cnn_config() (classifier_params.cpp). This
// duplication is the "short-term" state the plan calls out: a Phase 7 refactor is meant to
// replace both call sites with one `make_cnn_config(parameters)` function.
tl::CNNConfig build_cnn_config_via_parameter_loader(
  const ParameterLoader & loader, const std::string & label_path)
{
  tl::CNNConfig config;
  config.model_path = loader.get<std::string>("model_path");
  config.precision = loader.get<std::string>("precision");
  config.labels = read_label_lines(label_path);
  const auto mean_d = loader.get<std::vector<double>>("mean");
  const auto std_d = loader.get<std::vector<double>>("std");
  config.mean = std::vector<float>(mean_d.begin(), mean_d.end());
  config.std = std::vector<float>(std_d.begin(), std_d.end());
  return config;
}

}  // namespace

TEST(ClassifierParamsCrossCheckTest, ParameterLoaderMatchesDeclareCnnConfig)
{
  // Arrange: production param.yaml + the launch-injected keys (model_path/label_path are never in
  // param.yaml, see plan §2.4/§4 Q2). model_path is not actually opened by declare_cnn_config(), so
  // an arbitrary placeholder is enough for it; label_path must resolve to a real file.
  const std::string label_path = resolve_autoware_data_file(kLabelFilename);
  if (label_path.empty()) {
    GTEST_SKIP() << "traffic_light_classifier label file not found under autoware_data";
  }
  const std::string yaml_path =
    ament_index_cpp::get_package_share_directory("autoware_traffic_light_classifier") +
    "/config/car_traffic_light_classifier.param.yaml";
  constexpr char placeholder_model_path[] = "/dummy/model.onnx";

  // Act (a): production's path -- an rclcpp::Node with the yaml loaded as parameter overrides.
  rclcpp::NodeOptions node_options;
  node_options.arguments({"--ros-args", "--params-file", yaml_path});
  node_options.append_parameter_override("label_path", label_path);
  node_options.append_parameter_override("model_path", placeholder_model_path);
  auto node = std::make_shared<rclcpp::Node>("classifier_params_cross_check", node_options);
  const tl::CNNConfig config_from_node = tl::declare_cnn_config(node.get());

  // Act (b): the Component Test's path -- ParameterLoader, no rclcpp::Node.
  ParameterLoader loader;
  loader.merge_yaml_file(yaml_path);
  loader.set_override("label_path", rclcpp::ParameterValue(label_path));
  loader.set_override("model_path", rclcpp::ParameterValue(std::string(placeholder_model_path)));
  const tl::CNNConfig config_from_loader =
    build_cnn_config_via_parameter_loader(loader, label_path);

  // Assert: both paths build the identical config struct from the identical yaml.
  EXPECT_EQ(config_from_node.model_path, config_from_loader.model_path);
  EXPECT_EQ(config_from_node.precision, config_from_loader.precision);
  EXPECT_EQ(config_from_node.labels, config_from_loader.labels);
  EXPECT_EQ(config_from_node.mean, config_from_loader.mean);
  EXPECT_EQ(config_from_node.std, config_from_loader.std);

  // Assert: the LampRecognizer-only keys that a CNN classifier_type never declares (plan §2.4)
  // are exactly the keys ParameterLoader also sees as unread once the CNN config has been built --
  // i.e. this scenario's allow-list has no gap and no unexplained extra entry among them.
  const std::vector<std::string> unused = loader.unused_parameter_names();
  for (const char * lamp_only_key : {"score_threshold", "nms_threshold", "max_batch_size"}) {
    EXPECT_NE(std::find(unused.begin(), unused.end(), lamp_only_key), unused.end())
      << lamp_only_key << " should be unused when classifier_type selects CNN";
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
