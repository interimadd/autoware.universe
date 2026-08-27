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
  config.score_threshold =
    static_cast<float>(node->declare_parameter<double>(joined(prefix, "score_threshold")));
  config.nms_threshold =
    static_cast<float>(node->declare_parameter<double>(joined(prefix, "nms_threshold")));
  // Only fp16 models are used in this package, so precision and the int8-only knobs
  // (calibration_algorithm / dla_core_id / quantize_first_layer / quantize_last_layer /
  // clip_value / calibration_image_list_path) are fixed rather than exposed as parameters.
  // profile_per_layer is dev-only (may affect execution speed) and likewise fixed. gpu_id is
  // fixed to the default CUDA device: this package does not need per-node GPU selection.
  config.precision = "fp16";
  config.calibration_algorithm = "Entropy";
  config.dla_core_id = -1;
  config.quantize_first_layer = false;
  config.quantize_last_layer = false;
  config.profile_per_layer = false;
  config.clip_value = 6.0;
  config.calibration_image_list_path = "";
  config.gpu_id = 0;

  config.roi_labels = autoware::tensorrt_yolox::load_label_maps(label_path, roi_remap_path, "");
  // The traffic-light yolox model has no segmentation head: these are fixed rather than exposed
  // as parameters (plan §5.1).
  config.semseg_color_map = autoware::tensorrt_yolox::load_segmentation_colormap("");
  config.is_roi_overlap_semseg = false;
  config.is_publish_color_mask = false;
  config.overlap_roi_score_threshold = 0.0f;
  return config;
}

TrafficLightMapBasedDetectorConfig declare_map_based_detector_config()
{
  TrafficLightMapBasedDetectorConfig config;
  // Fixed rather than declared as parameters: unlike min_/max_timestamp_offset (declared in
  // declare_transform_sampling_config(), read from this same prefix), none of these vary between
  // deployments in practice -- the calibration-error margins (max_vibration_*) and the
  // range/angle cutoffs (max_detection_range / *_traffic_light_max_angle_range) use the same
  // defaults as autoware_traffic_light_map_based_detector's own config, with no per-vehicle or
  // per-camera override in use.
  config.max_vibration_pitch = 0.01745329251;
  config.max_vibration_yaw = 0.01745329251;
  config.max_vibration_height = 0.5;
  config.max_vibration_width = 0.5;
  config.max_vibration_depth = 0.5;
  config.max_detection_range = 200.0;
  config.car_traffic_light_max_angle_range = 40.0;
  config.pedestrian_traffic_light_max_angle_range = 80.0;
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
  const std::string & label_path, uint8_t classify_traffic_light_type)
{
  TrafficLightRecognitionClassifierConfig config;
  // Which classifier instance this is (car vs. pedestrian) is fixed by the caller's choice of
  // prefix, not read as a parameter -- see declare_classifier_config()'s declaration comment.
  config.classify_traffic_light_type = classify_traffic_light_type;
  config.over_exposure_threshold =
    node->declare_parameter<double>(joined(prefix, "over_exposure_threshold"));
  config.under_exposure_threshold =
    node->declare_parameter<double>(joined(prefix, "under_exposure_threshold"));

  config.cnn.model_path = model_path;
  // Only fp16 models are used in this package, so precision is fixed rather than exposed as a
  // parameter (see the same rationale in declare_whole_image_detector_config()). mean/std are the
  // per-channel (RGB) input normalization statistics the classifier model was trained with -- a
  // property of the model, not something to tune per deployment -- so they are fixed here too,
  // matching the ImageNet-style normalization both car and pedestrian classifier models use.
  config.cnn.precision = "fp16";
  config.cnn.labels = read_label_file(node, label_path);
  config.cnn.mean = {123.675f, 116.28f, 103.53f};
  config.cnn.std = {58.395f, 57.12f, 57.375f};
  return config;
}

}  // namespace autoware::traffic_light
