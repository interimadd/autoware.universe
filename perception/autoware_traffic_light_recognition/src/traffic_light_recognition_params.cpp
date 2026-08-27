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

#include "traffic_light_recognition_params.hpp"

#include <autoware/tensorrt_yolox/label.hpp>

#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::traffic_light
{
namespace
{
// Reads the label file into per-line labels, failing node construction fast (rather than leaving
// a classifier with an empty label table) on a misconfigured path. Mirrors
// classifier_params.cpp's read_label_file().
std::vector<std::string> read_label_file(rclcpp::Node * node, const std::string & filepath)
{
  std::ifstream labels_file(filepath);
  if (!labels_file.is_open()) {
    RCLCPP_ERROR(node->get_logger(), "Could not open label file. [%s]", filepath.c_str());
    throw std::runtime_error("Could not open label file: " + filepath);
  }
  std::vector<std::string> labels;
  std::string label;
  while (std::getline(labels_file, label)) {
    labels.push_back(label);
  }
  return labels;
}

std::string joined(const std::string & prefix, const std::string & key)
{
  return prefix + "." + key;
}
}  // namespace

autoware::tensorrt_yolox::TrtYoloXDetectorConfig declare_whole_image_detector_config(
  rclcpp::Node * node, const std::string & prefix, const std::string & model_path,
  const std::string & label_path, const std::string & roi_remap_path)
{
  autoware::tensorrt_yolox::TrtYoloXDetectorConfig config;
  config.model_path = model_path;
  config.precision = node->declare_parameter<std::string>(joined(prefix, "precision"));
  config.score_threshold =
    static_cast<float>(node->declare_parameter<double>(joined(prefix, "score_threshold")));
  config.nms_threshold =
    static_cast<float>(node->declare_parameter<double>(joined(prefix, "nms_threshold")));
  config.calibration_algorithm =
    node->declare_parameter<std::string>(joined(prefix, "calibration_algorithm"));
  config.dla_core_id = node->declare_parameter<int>(joined(prefix, "dla_core_id"));
  config.quantize_first_layer =
    node->declare_parameter<bool>(joined(prefix, "quantize_first_layer"));
  config.quantize_last_layer = node->declare_parameter<bool>(joined(prefix, "quantize_last_layer"));
  config.profile_per_layer = node->declare_parameter<bool>(joined(prefix, "profile_per_layer"));
  config.clip_value = node->declare_parameter<double>(joined(prefix, "clip_value"));
  config.calibration_image_list_path =
    node->declare_parameter<std::string>(joined(prefix, "calibration_image_list_path"));
  config.gpu_id = static_cast<uint8_t>(node->declare_parameter<int>(joined(prefix, "gpu_id")));

  config.roi_labels = autoware::tensorrt_yolox::load_label_maps(label_path, roi_remap_path, "");
  // The traffic-light yolox model has no segmentation head: these are fixed rather than exposed
  // as parameters (plan §5.1).
  config.semseg_color_map = autoware::tensorrt_yolox::load_segmentation_colormap("");
  config.is_roi_overlap_semseg = false;
  config.is_publish_color_mask = false;
  config.overlap_roi_score_threshold = 0.0f;
  return config;
}

TrafficLightMapBasedDetectorConfig declare_map_based_detector_config(
  rclcpp::Node * node, const std::string & prefix)
{
  TrafficLightMapBasedDetectorConfig config;
  config.max_vibration_pitch =
    node->declare_parameter<double>(joined(prefix, "max_vibration_pitch"));
  config.max_vibration_yaw = node->declare_parameter<double>(joined(prefix, "max_vibration_yaw"));
  config.max_vibration_height =
    node->declare_parameter<double>(joined(prefix, "max_vibration_height"));
  config.max_vibration_width =
    node->declare_parameter<double>(joined(prefix, "max_vibration_width"));
  config.max_vibration_depth =
    node->declare_parameter<double>(joined(prefix, "max_vibration_depth"));
  config.max_detection_range =
    node->declare_parameter<double>(joined(prefix, "max_detection_range"));
  config.car_traffic_light_max_angle_range =
    node->declare_parameter<double>(joined(prefix, "car_traffic_light_max_angle_range"));
  config.pedestrian_traffic_light_max_angle_range =
    node->declare_parameter<double>(joined(prefix, "pedestrian_traffic_light_max_angle_range"));
  return config;
}

TransformSamplingConfig declare_transform_sampling_config(
  rclcpp::Node * node, const std::string & prefix)
{
  TransformSamplingConfig config;
  config.min_timestamp_offset =
    node->declare_parameter<double>(joined(prefix, "min_timestamp_offset"));
  config.max_timestamp_offset =
    node->declare_parameter<double>(joined(prefix, "max_timestamp_offset"));
  return config;
}

TrafficLightRecognitionClassifierConfig declare_classifier_config(
  rclcpp::Node * node, const std::string & prefix, const std::string & model_path,
  const std::string & label_path)
{
  TrafficLightRecognitionClassifierConfig config;
  config.classify_traffic_light_type =
    static_cast<uint8_t>(node->declare_parameter<int>(joined(prefix, "traffic_light_type")));
  config.over_exposure_threshold =
    node->declare_parameter<double>(joined(prefix, "over_exposure_threshold"));
  config.under_exposure_threshold =
    node->declare_parameter<double>(joined(prefix, "under_exposure_threshold"));

  config.cnn.model_path = model_path;
  config.cnn.precision = node->declare_parameter<std::string>(joined(prefix, "precision"));
  config.cnn.labels = read_label_file(node, label_path);
  const auto mean = node->declare_parameter<std::vector<double>>(joined(prefix, "mean"));
  const auto std_dev = node->declare_parameter<std::vector<double>>(joined(prefix, "std"));
  config.cnn.mean = std::vector<float>(mean.begin(), mean.end());
  config.cnn.std = std::vector<float>(std_dev.begin(), std_dev.end());
  return config;
}

}  // namespace autoware::traffic_light
