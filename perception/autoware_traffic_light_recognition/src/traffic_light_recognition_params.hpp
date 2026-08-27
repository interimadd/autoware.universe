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

#ifndef TRAFFIC_LIGHT_RECOGNITION_PARAMS_HPP_
#define TRAFFIC_LIGHT_RECOGNITION_PARAMS_HPP_

// Node-side helpers that declare the pipeline parameters on a node and return the plain config
// structs the (ROS-free) TrafficLightRecognition core consumes. Lives at the node layer, not
// under include/, because declaring parameters is a Node concern; the core itself never touches
// rclcpp. Mirrors autoware_traffic_light_classifier's classifier_params.hpp / declare_cnn_config()
// pattern, and reads the same keys as the Component Test's build_*_config() helpers
// (run_traffic_light_pipeline_main.cpp).

#include <autoware/traffic_light_recognition/traffic_light_recognition.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>

namespace autoware::traffic_light
{

// Declares the whole_image_detector.* parameters under `prefix` on `node` and returns the
// resulting config. model_path / label_path / roi_remap_path are injected by the caller (launch
// argument, plan §5.1) rather than declared here, so the config file never hard-codes a
// $HOME/autoware_data path. semseg-only fields (is_roi_overlap_semseg / is_publish_color_mask /
// overlap_roi_score_threshold / semseg_color_map) are fixed here rather than exposed as
// parameters: the traffic-light yolox model has no segmentation head (plan §5.1).
autoware::tensorrt_yolox::TrtYoloXDetectorConfig declare_whole_image_detector_config(
  rclcpp::Node * node, const std::string & prefix, const std::string & model_path,
  const std::string & label_path, const std::string & roi_remap_path);

// Declares the map_based_detector.* parameters under `prefix` on `node` and returns the
// resulting config.
TrafficLightMapBasedDetectorConfig declare_map_based_detector_config(
  rclcpp::Node * node, const std::string & prefix);

// Declares the transform-sampling parameters (min_/max_timestamp_offset), read from the same
// `prefix` as declare_map_based_detector_config() (they live in the same param.yaml section).
TransformSamplingConfig declare_transform_sampling_config(
  rclcpp::Node * node, const std::string & prefix);

// Declares one classifier's parameters (car_classifier or pedestrian_classifier) under `prefix`
// on `node` and returns the resulting config. The same function handles both car and pedestrian:
// the only difference between them is the parameter prefix and the model/label files passed in.
// classifier_type is not read here -- it is fixed to CNN (plan §5.1) -- and score_threshold /
// nms_threshold / max_batch_size (CnnLampRecognizerConfig-only keys) are likewise not read.
TrafficLightRecognitionClassifierConfig declare_classifier_config(
  rclcpp::Node * node, const std::string & prefix, const std::string & model_path,
  const std::string & label_path);

}  // namespace autoware::traffic_light

#endif  // TRAFFIC_LIGHT_RECOGNITION_PARAMS_HPP_
