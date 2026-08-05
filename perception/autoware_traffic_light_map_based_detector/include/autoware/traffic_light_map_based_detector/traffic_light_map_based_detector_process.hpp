// Copyright 2025 TIER IV, Inc.
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

#ifndef AUTOWARE__TRAFFIC_LIGHT_MAP_BASED_DETECTOR__TRAFFIC_LIGHT_MAP_BASED_DETECTOR_PROCESS_HPP_
#define AUTOWARE__TRAFFIC_LIGHT_MAP_BASED_DETECTOR__TRAFFIC_LIGHT_MAP_BASED_DETECTOR_PROCESS_HPP_

#include <opencv2/core.hpp>
#include <rclcpp/time.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/LinearMath/Vector3.hpp>

#include <std_msgs/msg/header.hpp>
#include <tier4_perception_msgs/msg/traffic_light_roi_array.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <tf2/buffer_core.h>

#include <optional>
#include <string>
#include <vector>

#if __has_include(<image_geometry/pinhole_camera_model.hpp>)
#include <image_geometry/pinhole_camera_model.hpp>  // for ROS 2 Jazzy or newer
#else
#include <image_geometry/pinhole_camera_model.h>  // for ROS 2 Humble or older
#endif

namespace autoware::traffic_light
{

/// A map->camera transform sampled at a specific point in time.
struct StampedTransform
{
  rclcpp::Time stamp;
  tf2::Transform transform;
};

/// Configures how sample_map_to_camera_transforms() spreads its samples around the camera_info
/// header stamp. Offsets are seconds relative to that stamp (min_timestamp_offset is typically
/// negative, sampling into the past).
struct TransformSamplingConfig
{
  double min_timestamp_offset;
  double max_timestamp_offset;
};

/// Samples the map->camera transform every 0.01 s from
/// camera_header.stamp + min_timestamp_offset to camera_header.stamp + max_timestamp_offset,
/// plus the transform at the exact header stamp. Mirrors the sampling loop originally in
/// MapBasedDetector::camera_info_callback(). Returns std::nullopt when the transform at the
/// exact header stamp is unavailable, matching the original behavior of aborting the callback
/// in that case (samples for the offset range that failed to resolve are simply omitted).
std::optional<std::vector<StampedTransform>> sample_map_to_camera_transforms(
  const tf2::BufferCore & tf_buffer, const std_msgs::msg::Header & camera_header,
  const TransformSamplingConfig & config);

namespace utils
{

/// Looks up the map->frame_id transform at `time` without waiting. Returns std::nullopt on any
/// tf2::TransformException (e.g. the transform is not yet, or no longer, available).
std::optional<tf2::Transform> lookup_map_to_camera_transform(
  const tf2::BufferCore & tf_buffer, const rclcpp::Time & time, const std::string & frame_id);

cv::Point2d calc_raw_image_point_from_point_3d(
  const image_geometry::PinholeCameraModel & pinhole_camera_model, const cv::Point3d & point3d);

cv::Point2d calc_raw_image_point_from_point_3d(
  const image_geometry::PinholeCameraModel & pinhole_camera_model, const tf2::Vector3 & point3d);

void round_in_image_frame(const uint32_t & width, const uint32_t & height, cv::Point2d & point);

bool is_in_distance_range(
  const tf2::Vector3 & p1, const tf2::Vector3 & p2, const double max_distance_range);

bool is_in_angle_range(
  const double & tl_yaw, const double & camera_yaw, const double max_angle_range);

bool is_in_image_frame(
  const image_geometry::PinholeCameraModel & pinhole_camera_model, const tf2::Vector3 & point);

// Calculated in the camera optical frame but yaw and pitch are in the camera frame
tf2::Vector3 get_vibration_margin(
  const double depth, const double margin_pitch, const double margin_yaw,
  const double margin_height, const double margin_width, const double margin_depth);

void compute_bounding_roi(
  const uint32_t & width, const uint32_t & height,
  const std::vector<tier4_perception_msgs::msg::TrafficLightRoi> & rois,
  tier4_perception_msgs::msg::TrafficLightRoi & max_roi);

double get_traffic_light_yaw(const lanelet::ConstLineString3d & traffic_light);

double get_camera_yaw(const tf2::Transform & tf_map2camera);

}  // namespace utils
}  // namespace autoware::traffic_light

#endif  // AUTOWARE__TRAFFIC_LIGHT_MAP_BASED_DETECTOR__TRAFFIC_LIGHT_MAP_BASED_DETECTOR_PROCESS_HPP_
