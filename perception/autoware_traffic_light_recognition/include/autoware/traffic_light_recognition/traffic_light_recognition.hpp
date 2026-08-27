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

#ifndef AUTOWARE__TRAFFIC_LIGHT_RECOGNITION__TRAFFIC_LIGHT_RECOGNITION_HPP_
#define AUTOWARE__TRAFFIC_LIGHT_RECOGNITION__TRAFFIC_LIGHT_RECOGNITION_HPP_

#include <autoware/tensorrt_yolox/tensorrt_yolox_detector.hpp>
#include <autoware/traffic_light_classifier/classifier/cnn_classifier.hpp>
#include <autoware/traffic_light_classifier/traffic_light_classifier.hpp>
#include <autoware/traffic_light_map_based_detector/traffic_light_map_based_detector.hpp>
#include <autoware/traffic_light_map_based_detector/traffic_light_map_based_detector_process.hpp>
#include <tl_expected/expected.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tier4_perception_msgs/msg/traffic_light_array.hpp>
#include <tier4_perception_msgs/msg/traffic_light_roi_array.hpp>

#include <tf2/buffer_core.h>

#include <cstdint>
#include <optional>
#include <string>

// Single-Node composition of the whole_image_detection front-end pipeline (plan §0):
//   map_based_detector -> whole_image_detector(yolox) -> selector
//     -> car_classifier / pedestrian_classifier -> category_merger
//
// This header (and its .cpp) is a straight port of
// autoware_traffic_light_component_test's TrafficLightRecognitionPipeline (plan §3): same
// composition, same run() body, renamed to be a first-class production API. The Component Test
// package keeps using its own copy until Phase 4 (plan §5.4) replaces it with this one.

namespace autoware::traffic_light
{

// Configuration for one TrafficLightClassifier + its backend.
// classifier_type is fixed to CNN for both car and pedestrian
// (traffic_light_component_test_plan.md §5.6 / this plan §5.1): the only backend config held here
// is CNNConfig. A future classifier_type would add a variant here.
struct TrafficLightRecognitionClassifierConfig
{
  CNNConfig cnn;
  // tier4_perception_msgs::msg::TrafficLight::CAR_TRAFFIC_LIGHT or ::PEDESTRIAN_TRAFFIC_LIGHT.
  uint8_t classify_traffic_light_type = 0;
  double over_exposure_threshold = 0.85;
  double under_exposure_threshold = -0.83;
};

// Configuration for one camera's worth of the whole_image_detection front-end. Decoding the input
// image is out of scope (plan §0): the input is always an already-decoded sensor_msgs::msg::Image.
struct TrafficLightRecognitionConfig
{
  autoware::tensorrt_yolox::TrtYoloXDetectorConfig whole_image_detector;
  TrafficLightMapBasedDetectorConfig map_based_detector;
  // Read from the same parameter prefix as map_based_detector (min_/max_timestamp_offset drive
  // the tf sampling run() does before each detection); see declare_transform_sampling_config().
  TransformSamplingConfig transform_sampling;
  TrafficLightRecognitionClassifierConfig car_classifier;
  TrafficLightRecognitionClassifierConfig pedestrian_classifier;
};

// run()'s output: the merged (car + pedestrian) signals and the selected ROIs that produced them.
struct TrafficLightRecognitionResult
{
  tier4_perception_msgs::msg::TrafficLightArray merged_signals;
  tier4_perception_msgs::msg::TrafficLightRoiArray selected_rois;
};

// The signal-recognition composition for one camera: map->camera transform sampling +
// whole-image yolox detection + map-based rough/expect ROI detection + ROI selection + car /
// pedestrian CNN classification + category merge, run synchronously for one frame at a time.
class TrafficLightRecognition
{
public:
  // `tf_buffer` must already hold every map->camera transform the frames passed to run() will
  // need and must outlive this object: the owning Node holds the tf2_ros::Buffer that backs it,
  // which by construction outlives the sync callback that drives run() (plan §3.2).
  TrafficLightRecognition(
    const TrafficLightRecognitionConfig & config,
    const autoware_map_msgs::msg::LaneletMapBin & map_msg, const tf2::BufferCore & tf_buffer);

  // Restricts map_based_detector's target traffic lights to those on `route_msg`.
  std::optional<SetRouteError> set_route(
    const autoware_planning_msgs::msg::LaneletRoute & route_msg);

  // Runs one frame end-to-end. The map->camera transforms this frame needs are not passed in:
  // run() samples them itself out of the tf buffer given at construction, keyed by
  // `camera_info.header` -- exactly as MapBasedDetector::camera_info_callback() does. Failure
  // modes are the ones the underlying cores already report (yolox inference / classifier image
  // decoding) plus a map->camera transform that cannot be resolved at the frame's stamp, which
  // aborts the frame before any detection runs.
  tl::expected<TrafficLightRecognitionResult, std::string> run(
    const sensor_msgs::msg::Image & image, const sensor_msgs::msg::CameraInfo & camera_info);

private:
  autoware::tensorrt_yolox::TrtYoloXDetector whole_image_detector_;
  TrafficLightMapBasedDetector map_based_detector_;
  TrafficLightClassifier car_classifier_;
  TrafficLightClassifier pedestrian_classifier_;
  TransformSamplingConfig transform_sampling_config_;
  const tf2::BufferCore & tf_buffer_;
};

}  // namespace autoware::traffic_light

#endif  // AUTOWARE__TRAFFIC_LIGHT_RECOGNITION__TRAFFIC_LIGHT_RECOGNITION_HPP_
