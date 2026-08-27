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

// Configuration for one camera's worth of the whole_image_detection front-end. Decoding the input
// image is out of scope (plan §0): the input is always an already-decoded sensor_msgs::msg::Image.
//
// Deliberately flat, and deliberately holding only the values actually supplied via ROS 2
// parameters (declare_config(), traffic_light_recognition_node.cpp) or, for the *_model_path /
// *_label_path fields, launch arguments (plan §5.1 keeps model/label paths out of the versioned
// param.yaml). Everything else the underlying cores need -- precision, mean/std, gpu_id,
// classify_traffic_light_type, the map_based_detector calibration-error margins and range/angle
// cutoffs, the semseg-only yolox fields, ... -- has no per-deployment override in practice, so it
// is fixed inside TrafficLightRecognition's constructor and build_engines() below (see their
// definitions in traffic_light_recognition.cpp), not read from a parameter by the Node.
struct TrafficLightRecognitionConfig
{
  std::string whole_image_detector_model_path;
  std::string whole_image_detector_label_path;
  std::string whole_image_detector_roi_remap_path;
  float whole_image_detector_score_threshold = 0.0f;
  float whole_image_detector_nms_threshold = 0.0f;

  // Drive the map->camera tf sampling run() does before each detection (plan §5.1); read from
  // the map_based_detector.* parameter prefix even though nothing else in that prefix is a
  // parameter any more (see declare_config()).
  double min_timestamp_offset = 0.0;
  double max_timestamp_offset = 0.0;

  std::string car_classifier_model_path;
  std::string car_classifier_label_path;
  double car_classifier_over_exposure_threshold = 0.85;
  double car_classifier_under_exposure_threshold = -0.83;

  std::string pedestrian_classifier_model_path;
  std::string pedestrian_classifier_label_path;
  double pedestrian_classifier_over_exposure_threshold = 0.85;
  double pedestrian_classifier_under_exposure_threshold = -0.83;
};

// Builds (and discards) the whole-image detector's and both classifiers' TensorRT engines from
// `config`, without a map -- used by build_only (plan §4.4), where the Node exits once engine
// construction succeeds rather than constructing a full TrafficLightRecognition. Throws on
// failure, same as the underlying engine constructors.
void build_engines(const TrafficLightRecognitionConfig & config);

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
