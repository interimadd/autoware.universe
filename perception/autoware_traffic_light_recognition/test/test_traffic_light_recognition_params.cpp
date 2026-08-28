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
// Tests for declare_config() (plan §5.3, §6): reads the package's own default
// config/traffic_light_recognition.param.yaml through an rclcpp::Node the same way the production
// Node does, and checks the resulting flat TrafficLightRecognitionConfig matches what that yaml
// (plus the launch-injected model/label paths) says. No GPU / TensorRT engine is built here:
// declare_config() only declares parameters, it never touches a model or label file itself --
// that (and every fixed, non-parameter value) is TrafficLightRecognition's job
// (traffic_light_recognition.cpp), exercised by test_traffic_light_recognition.cpp instead.
//
// The important assertion here is that car_classifier / pedestrian_classifier are read from
// distinct parameter prefixes: a bug that read the same subtree twice would make
// car_classifier_model_path / pedestrian_classifier_model_path collapse to the same value.
//

#include "../src/traffic_light_recognition_node.hpp"

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
  return ament_index_cpp::get_package_share_directory("autoware_traffic_light_recognition") +
         "/config/traffic_light_recognition.param.yaml";
}

// Builds a node with the package's default config.yaml plus the launch-injected model/label
// parameters that config.yaml deliberately omits (plan §5.1). declare_config() never opens these
// paths itself, so arbitrary (possibly non-existent) paths are fine.
std::shared_ptr<rclcpp::Node> make_test_node()
{
  std::vector<std::string> args{
    "--ros-args",
    "--params-file",
    default_config_path(),
    "-p",
    "whole_image_detector.model_path:=/tmp/does_not_need_to_exist.onnx",
    "-p",
    "whole_image_detector.label_path:=/tmp/does_not_need_to_exist_labels.txt",
    "-p",
    "car_classifier.model_path:=/tmp/does_not_need_to_exist_car.onnx",
    "-p",
    "car_classifier.label_path:=/tmp/does_not_need_to_exist_car_labels.txt",
    "-p",
    "pedestrian_classifier.model_path:=/tmp/does_not_need_to_exist_ped.onnx",
    "-p",
    "pedestrian_classifier.label_path:=/tmp/does_not_need_to_exist_ped_labels.txt",
  };
  rclcpp::NodeOptions options;
  options.arguments(args);
  return std::make_shared<rclcpp::Node>("test_traffic_light_recognition_params", options);
}

TEST(ParamsTest, ConfigMatchesDefaultYamlAndInjectedPaths)
{
  // Arrange
  auto node = make_test_node();

  // Act
  const auto config = tl::declare_config(node.get());

  // Assert -- whole_image_detector.*
  EXPECT_EQ(config.whole_image_detector_model_path, "/tmp/does_not_need_to_exist.onnx");
  EXPECT_EQ(config.whole_image_detector_label_path, "/tmp/does_not_need_to_exist_labels.txt");
  EXPECT_EQ(config.whole_image_detector_roi_remap_path, "");
  EXPECT_FLOAT_EQ(config.whole_image_detector_score_threshold, 0.35f);
  EXPECT_FLOAT_EQ(config.whole_image_detector_nms_threshold, 0.7f);

  // Assert -- map_based_detector.*
  EXPECT_DOUBLE_EQ(config.min_timestamp_offset, -0.3);
  EXPECT_DOUBLE_EQ(config.max_timestamp_offset, 0.0);

  // Assert -- car_classifier.* / pedestrian_classifier.*, read from distinct prefixes.
  EXPECT_EQ(config.car_classifier_model_path, "/tmp/does_not_need_to_exist_car.onnx");
  EXPECT_EQ(config.car_classifier_label_path, "/tmp/does_not_need_to_exist_car_labels.txt");

  EXPECT_EQ(config.pedestrian_classifier_model_path, "/tmp/does_not_need_to_exist_ped.onnx");
  EXPECT_EQ(config.pedestrian_classifier_label_path, "/tmp/does_not_need_to_exist_ped_labels.txt");

  EXPECT_NE(config.car_classifier_model_path, config.pedestrian_classifier_model_path);

  // Assert -- over_exposure_threshold / under_exposure_threshold, shared by both classifiers
  // (top-level, not under either classifier's prefix).
  EXPECT_DOUBLE_EQ(config.over_exposure_threshold, 0.85);
  EXPECT_DOUBLE_EQ(config.under_exposure_threshold, -0.83);
}

}  // namespace

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
