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
// Tests for the declare_*_config() family (plan §5.3, §6): reads the package's own default
// config/traffic_light_recognition.param.yaml through an rclcpp::Node the same way the production
// Node does, and checks the resulting config structs match what that yaml says. No GPU / TensorRT
// engine is built here: declare_*_config() only declares parameters and reads label files.
//
// The single most important assertion (plan §6) is that car_classifier / pedestrian_classifier
// are read from distinct parameter prefixes: feeding the same declare_classifier_config() prefix
// argument twice must not silently collapse the two into one config.
//

#include "../src/traffic_light_recognition_params.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tier4_perception_msgs/msg/traffic_light.hpp>

#include <gtest/gtest.h>

#include <fstream>
#include <memory>
#include <string>
#include <vector>

namespace
{
namespace tl = autoware::traffic_light;

// A minimal, always-openable label file: declare_classifier_config() /
// declare_whole_image_detector_config() read this file's lines but never require any particular
// content, and neither function touches the model file itself (that only happens inside the
// TensorRT engine constructors, which this test never calls).
std::string write_temp_label_file()
{
  const std::string path = "/tmp/test_traffic_light_recognition_params_labels.txt";
  std::ofstream file(path);
  file << "red\n";
  file << "yellow\n";
  file << "green\n";
  return path;
}

std::string default_config_path()
{
  return ament_index_cpp::get_package_share_directory("autoware_traffic_light_recognition") +
         "/config/traffic_light_recognition.param.yaml";
}

// Builds a node with the package's default config.yaml plus the launch-injected model/label
// parameters that config.yaml deliberately omits (plan §5.1). model paths are never opened by
// declare_*_config(), so an arbitrary (possibly non-existent) path is fine.
std::shared_ptr<rclcpp::Node> make_test_node(const std::string & label_path)
{
  std::vector<std::string> args{
    "--ros-args",
    "--params-file",
    default_config_path(),
    "-p",
    "whole_image_detector.model_path:=/tmp/does_not_need_to_exist.onnx",
    "-p",
    "whole_image_detector.label_path:=" + label_path,
    "-p",
    "car_classifier.model_path:=/tmp/does_not_need_to_exist_car.onnx",
    "-p",
    "car_classifier.label_path:=" + label_path,
    "-p",
    "pedestrian_classifier.model_path:=/tmp/does_not_need_to_exist_ped.onnx",
    "-p",
    "pedestrian_classifier.label_path:=" + label_path,
  };
  rclcpp::NodeOptions options;
  options.arguments(args);
  return std::make_shared<rclcpp::Node>("test_traffic_light_recognition_params", options);
}

class ParamsTest : public ::testing::Test
{
protected:
  void SetUp() override { label_path_ = write_temp_label_file(); }
  std::string label_path_;
};

// declare_whole_image_detector_config() reads whole_image_detector.* from the package default
// config and produces a TrtYoloXDetectorConfig matching those values.
TEST_F(ParamsTest, WholeImageDetectorConfigMatchesDefaultYaml)
{
  // Arrange
  auto node = make_test_node(label_path_);

  // Act
  const auto config = tl::declare_whole_image_detector_config(
    node.get(), "whole_image_detector", "/tmp/does_not_need_to_exist.onnx", label_path_, "");

  // Assert
  EXPECT_EQ(config.precision, "fp16");
  EXPECT_FLOAT_EQ(config.score_threshold, 0.35f);
  EXPECT_FLOAT_EQ(config.nms_threshold, 0.7f);
  EXPECT_EQ(config.calibration_algorithm, "Entropy");
  EXPECT_EQ(config.dla_core_id, -1);
  EXPECT_FALSE(config.quantize_first_layer);
  EXPECT_FALSE(config.quantize_last_layer);
  EXPECT_FALSE(config.profile_per_layer);
  EXPECT_DOUBLE_EQ(config.clip_value, 6.0);
  EXPECT_EQ(config.gpu_id, 0);
  // semseg-only fields are fixed, not read from parameters (plan §5.1).
  EXPECT_FALSE(config.is_roi_overlap_semseg);
  EXPECT_FALSE(config.is_publish_color_mask);
  EXPECT_TRUE(config.semseg_color_map.empty());
}

TEST_F(ParamsTest, MapBasedDetectorConfigMatchesDefaultYaml)
{
  // Arrange
  auto node = make_test_node(label_path_);

  // Act
  const auto config = tl::declare_map_based_detector_config(node.get(), "map_based_detector");

  // Assert
  EXPECT_DOUBLE_EQ(config.max_vibration_pitch, 0.01745329251);
  EXPECT_DOUBLE_EQ(config.max_vibration_yaw, 0.01745329251);
  EXPECT_DOUBLE_EQ(config.max_vibration_height, 0.5);
  EXPECT_DOUBLE_EQ(config.max_vibration_width, 0.5);
  EXPECT_DOUBLE_EQ(config.max_vibration_depth, 0.5);
  EXPECT_DOUBLE_EQ(config.max_detection_range, 200.0);
  EXPECT_DOUBLE_EQ(config.car_traffic_light_max_angle_range, 40.0);
  EXPECT_DOUBLE_EQ(config.pedestrian_traffic_light_max_angle_range, 80.0);
}

TEST_F(ParamsTest, TransformSamplingConfigMatchesDefaultYaml)
{
  // Arrange
  auto node = make_test_node(label_path_);

  // Act
  const auto config = tl::declare_transform_sampling_config(node.get(), "map_based_detector");

  // Assert
  EXPECT_DOUBLE_EQ(config.min_timestamp_offset, -0.3);
  EXPECT_DOUBLE_EQ(config.max_timestamp_offset, 0.0);
}

// The central assertion (plan §6): declare_classifier_config() with prefix "car_classifier"
// and with prefix "pedestrian_classifier" on the SAME node must read distinct parameter subtrees,
// most visibly in classify_traffic_light_type (0 vs. 1). A bug that ignored `prefix` and always
// read the top-level keys (or read the same subtree twice) would make this test fail by producing
// two identical configs.
TEST_F(ParamsTest, CarAndPedestrianClassifierPrefixesAreDistinct)
{
  // Arrange
  auto node = make_test_node(label_path_);

  // Act
  const auto car_config = tl::declare_classifier_config(
    node.get(), "car_classifier", "/tmp/does_not_need_to_exist_car.onnx", label_path_);
  const auto pedestrian_config = tl::declare_classifier_config(
    node.get(), "pedestrian_classifier", "/tmp/does_not_need_to_exist_ped.onnx", label_path_);

  // Assert -- prefix separation is what makes these differ.
  EXPECT_EQ(
    car_config.classify_traffic_light_type,
    tier4_perception_msgs::msg::TrafficLight::CAR_TRAFFIC_LIGHT);
  EXPECT_EQ(
    pedestrian_config.classify_traffic_light_type,
    tier4_perception_msgs::msg::TrafficLight::PEDESTRIAN_TRAFFIC_LIGHT);
  EXPECT_NE(car_config.classify_traffic_light_type, pedestrian_config.classify_traffic_light_type);

  // Both share the same non-prefix-dependent defaults from the package config.
  for (const auto & config : {car_config, pedestrian_config}) {
    EXPECT_EQ(config.cnn.precision, "fp16");
    EXPECT_DOUBLE_EQ(config.over_exposure_threshold, 0.85);
    EXPECT_DOUBLE_EQ(config.under_exposure_threshold, -0.83);
    ASSERT_EQ(config.cnn.mean.size(), 3u);
    EXPECT_FLOAT_EQ(config.cnn.mean[0], 123.675f);
    ASSERT_EQ(config.cnn.std.size(), 3u);
    EXPECT_FLOAT_EQ(config.cnn.std[0], 58.395f);
    EXPECT_EQ(config.cnn.labels.size(), 3u);
  }

  // model_path is threaded straight through, per classifier.
  EXPECT_EQ(car_config.cnn.model_path, "/tmp/does_not_need_to_exist_car.onnx");
  EXPECT_EQ(pedestrian_config.cnn.model_path, "/tmp/does_not_need_to_exist_ped.onnx");
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
