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

#include "autoware/traffic_light_recognition/traffic_light_recognition.hpp"

#include <autoware/tensorrt_yolox/label.hpp>
#include <autoware/traffic_light_category_merger/traffic_light_category_merger.hpp>
#include <autoware/traffic_light_selector/traffic_light_selector.hpp>

#include <tier4_perception_msgs/msg/traffic_light.hpp>

#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace autoware::traffic_light
{
namespace
{
// Everything one CNNClassifier backend needs, with every fixed (non-parameter) field already
// filled in. Internal-only: the Node's declare_config() (traffic_light_recognition_node.cpp)
// never sees this type, only the flat, parameter-sourced TrafficLightRecognitionConfig.
struct ClassifierConfig
{
  CNNConfig cnn;
  // tier4_perception_msgs::msg::TrafficLight::CAR_TRAFFIC_LIGHT or ::PEDESTRIAN_TRAFFIC_LIGHT.
  uint8_t classify_traffic_light_type = 0;
  double over_exposure_threshold = 0.85;
  double under_exposure_threshold = -0.83;
};

// Reads the label file into per-line labels. Throws directly rather than going through
// RCLCPP_ERROR first (unlike autoware_traffic_light_classifier's classifier_params.cpp
// read_label_file(), which logs before throwing): this core never touches rclcpp.
std::vector<std::string> read_label_file(const std::string & filepath)
{
  std::ifstream labels_file(filepath);
  if (!labels_file.is_open()) {
    throw std::runtime_error("Could not open label file: " + filepath);
  }
  std::vector<std::string> labels;
  std::string label;
  while (std::getline(labels_file, label)) {
    labels.push_back(label);
  }
  return labels;
}

autoware::tensorrt_yolox::TrtYoloXDetectorConfig make_whole_image_detector_config(
  const TrafficLightRecognitionConfig & config)
{
  autoware::tensorrt_yolox::TrtYoloXDetectorConfig detector_config;
  detector_config.model_path = config.whole_image_detector_model_path;
  detector_config.score_threshold = config.whole_image_detector_score_threshold;
  detector_config.nms_threshold = config.whole_image_detector_nms_threshold;
  // Only fp16 models are used in this package, so precision and the int8-only knobs
  // (calibration_algorithm / dla_core_id / quantize_first_layer / quantize_last_layer /
  // clip_value / calibration_image_list_path) are fixed rather than exposed as parameters.
  // profile_per_layer is dev-only (may affect execution speed) and likewise fixed. gpu_id is
  // fixed to the default CUDA device: this package does not need per-node GPU selection.
  detector_config.precision = "fp16";
  detector_config.calibration_algorithm = "Entropy";
  detector_config.dla_core_id = -1;
  detector_config.quantize_first_layer = false;
  detector_config.quantize_last_layer = false;
  detector_config.profile_per_layer = false;
  detector_config.clip_value = 6.0;
  detector_config.calibration_image_list_path = "";
  detector_config.gpu_id = 0;

  detector_config.roi_labels = autoware::tensorrt_yolox::load_label_maps(
    config.whole_image_detector_label_path, config.whole_image_detector_roi_remap_path, "");
  // The traffic-light yolox model has no segmentation head: these are fixed rather than exposed
  // as parameters (plan §5.1).
  detector_config.semseg_color_map = autoware::tensorrt_yolox::load_segmentation_colormap("");
  detector_config.is_roi_overlap_semseg = false;
  detector_config.is_publish_color_mask = false;
  detector_config.overlap_roi_score_threshold = 0.0f;
  return detector_config;
}

TrafficLightMapBasedDetectorConfig make_map_based_detector_config()
{
  TrafficLightMapBasedDetectorConfig config;
  // Fixed rather than exposed as parameters: unlike min_/max_timestamp_offset (part of the
  // public TrafficLightRecognitionConfig, see make_transform_sampling_config()), none of these
  // vary between deployments in practice -- the calibration-error margins (max_vibration_*) and
  // the range/angle cutoffs (max_detection_range / *_traffic_light_max_angle_range) use the same
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

TransformSamplingConfig make_transform_sampling_config(const TrafficLightRecognitionConfig & config)
{
  TransformSamplingConfig sampling_config;
  sampling_config.min_timestamp_offset = config.min_timestamp_offset;
  sampling_config.max_timestamp_offset = config.max_timestamp_offset;
  return sampling_config;
}

// Builds one classifier's config. The same function handles both car and pedestrian: the only
// difference between them is the model/label files, the exposure thresholds, and
// `classify_traffic_light_type`, all passed in by the caller. precision is fixed (this package
// only ever runs fp16 models); mean/std are the per-channel (RGB) input normalization statistics
// the classifier model was trained with -- a property of the model, not something to tune per
// deployment -- so they are fixed too, matching the ImageNet-style normalization both car and
// pedestrian classifier models use.
ClassifierConfig make_classifier_config(
  const std::string & model_path, const std::string & label_path, double over_exposure_threshold,
  double under_exposure_threshold, uint8_t classify_traffic_light_type)
{
  ClassifierConfig classifier_config;
  classifier_config.classify_traffic_light_type = classify_traffic_light_type;
  classifier_config.over_exposure_threshold = over_exposure_threshold;
  classifier_config.under_exposure_threshold = under_exposure_threshold;
  classifier_config.cnn.model_path = model_path;
  classifier_config.cnn.precision = "fp16";
  classifier_config.cnn.labels = read_label_file(label_path);
  classifier_config.cnn.mean = {123.675f, 116.28f, 103.53f};
  classifier_config.cnn.std = {58.395f, 57.12f, 57.375f};
  return classifier_config;
}

ClassifierConfig make_car_classifier_config(const TrafficLightRecognitionConfig & config)
{
  return make_classifier_config(
    config.car_classifier_model_path, config.car_classifier_label_path,
    config.car_classifier_over_exposure_threshold, config.car_classifier_under_exposure_threshold,
    tier4_perception_msgs::msg::TrafficLight::CAR_TRAFFIC_LIGHT);
}

ClassifierConfig make_pedestrian_classifier_config(const TrafficLightRecognitionConfig & config)
{
  return make_classifier_config(
    config.pedestrian_classifier_model_path, config.pedestrian_classifier_label_path,
    config.pedestrian_classifier_over_exposure_threshold,
    config.pedestrian_classifier_under_exposure_threshold,
    tier4_perception_msgs::msg::TrafficLight::PEDESTRIAN_TRAFFIC_LIGHT);
}

TrafficLightClassifier make_classifier(const ClassifierConfig & config)
{
  // classifier_type is fixed to CNN for both car and pedestrian; a future classifier_type would
  // branch here the way traffic_light_classifier_node.cpp does.
  auto backend = std::make_shared<CNNClassifier>(config.cnn);
  return TrafficLightClassifier(
    std::move(backend), config.classify_traffic_light_type, config.over_exposure_threshold,
    config.under_exposure_threshold);
}
}  // namespace

void build_engines(const TrafficLightRecognitionConfig & config)
{
  [[maybe_unused]] autoware::tensorrt_yolox::TrtYoloXDetector whole_image_detector(
    make_whole_image_detector_config(config));
  [[maybe_unused]] CNNClassifier car_backend(make_car_classifier_config(config).cnn);
  [[maybe_unused]] CNNClassifier pedestrian_backend(make_pedestrian_classifier_config(config).cnn);
}

TrafficLightRecognition::TrafficLightRecognition(
  const TrafficLightRecognitionConfig & config,
  const autoware_map_msgs::msg::LaneletMapBin & map_msg, const tf2::BufferCore & tf_buffer)
: whole_image_detector_(make_whole_image_detector_config(config)),
  map_based_detector_(make_map_based_detector_config(), map_msg),
  car_classifier_(make_classifier(make_car_classifier_config(config))),
  pedestrian_classifier_(make_classifier(make_pedestrian_classifier_config(config))),
  transform_sampling_config_(make_transform_sampling_config(config)),
  tf_buffer_(tf_buffer)
{
}

std::optional<SetRouteError> TrafficLightRecognition::set_route(
  const autoware_planning_msgs::msg::LaneletRoute & route_msg)
{
  return map_based_detector_.set_route(route_msg);
}

tl::expected<TrafficLightRecognitionResult, std::string> TrafficLightRecognition::run(
  const sensor_msgs::msg::Image & image, const sensor_msgs::msg::CameraInfo & camera_info)
{
  // Sampled before anything else, mirroring MapBasedDetector::camera_info_callback(): a frame
  // whose map->camera transform cannot be resolved at the exact camera_info stamp is dropped
  // outright there, so nothing downstream -- not even the whole-image detection, which does not
  // depend on tf -- runs for it here either.
  const auto tf_map2camera_samples =
    sample_map_to_camera_transforms(tf_buffer_, camera_info.header, transform_sampling_config_);
  if (!tf_map2camera_samples) {
    return tl::make_unexpected(
      "map->camera transform unavailable for frame '" + camera_info.header.frame_id + "'");
  }

  const auto detected = whole_image_detector_.detect(image);
  if (!detected) {
    return tl::make_unexpected("whole_image_detector failed: " + detected.error());
  }

  const auto map_based_result = map_based_detector_.detect(*tf_map2camera_samples, camera_info);

  const auto selected_rois = select(
    detected->objects, map_based_result.rough_rois, map_based_result.expect_rois, camera_info);

  const auto car_result = car_classifier_.classify_image(image, selected_rois);
  if (!car_result) {
    return tl::make_unexpected("car classifier failed: " + car_result.error());
  }

  const auto pedestrian_result = pedestrian_classifier_.classify_image(image, selected_rois);
  if (!pedestrian_result) {
    return tl::make_unexpected("pedestrian classifier failed: " + pedestrian_result.error());
  }

  const auto merged_signals =
    TrafficLightCategoryMerger::merge(car_result->signals, pedestrian_result->signals);

  TrafficLightRecognitionResult result;
  result.merged_signals = merged_signals;
  result.selected_rois = selected_rois;
  return result;
}

}  // namespace autoware::traffic_light
