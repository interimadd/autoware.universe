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

#include <autoware/traffic_light_category_merger/traffic_light_category_merger.hpp>
#include <autoware/traffic_light_selector/traffic_light_selector.hpp>

#include <memory>
#include <string>
#include <utility>

namespace autoware::traffic_light
{
namespace
{
TrafficLightClassifier make_classifier(const TrafficLightRecognitionClassifierConfig & config)
{
  // classifier_type is fixed to CNN for both car and pedestrian; a future classifier_type would
  // branch here the way traffic_light_classifier_node.cpp does.
  auto backend = std::make_shared<CNNClassifier>(config.cnn);
  return TrafficLightClassifier(
    std::move(backend), config.classify_traffic_light_type, config.over_exposure_threshold,
    config.under_exposure_threshold);
}
}  // namespace

TrafficLightRecognition::TrafficLightRecognition(
  const TrafficLightRecognitionConfig & config,
  const autoware_map_msgs::msg::LaneletMapBin & map_msg, const tf2::BufferCore & tf_buffer)
: whole_image_detector_(config.whole_image_detector),
  map_based_detector_(config.map_based_detector, map_msg),
  car_classifier_(make_classifier(config.car_classifier)),
  pedestrian_classifier_(make_classifier(config.pedestrian_classifier)),
  transform_sampling_config_(config.transform_sampling),
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
