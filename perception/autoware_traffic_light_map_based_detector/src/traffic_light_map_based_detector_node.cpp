// Copyright 2023 TIER IV, Inc.
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

#define EIGEN_MPL2_ONLY

#include "traffic_light_map_based_detector_node.hpp"

#include <autoware/traffic_light_utils/traffic_light_utils.hpp>

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::traffic_light
{
MapBasedDetector::MapBasedDetector(const rclcpp::NodeOptions & node_options)
: Node("traffic_light_map_based_detector", node_options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  using std::placeholders::_1;

  // detector config
  detector_config_ = {
    this->declare_parameter<double>("max_vibration_pitch"),
    this->declare_parameter<double>("max_vibration_yaw"),
    this->declare_parameter<double>("max_vibration_height"),
    this->declare_parameter<double>("max_vibration_width"),
    this->declare_parameter<double>("max_vibration_depth"),
    this->declare_parameter<double>("max_detection_range"),
    this->declare_parameter<double>("car_traffic_light_max_angle_range"),
    this->declare_parameter<double>("pedestrian_traffic_light_max_angle_range")};
  // transform sampling config
  transform_sampling_config_ = {
    this->declare_parameter<double>("min_timestamp_offset"),
    this->declare_parameter<double>("max_timestamp_offset")};

  if (
    transform_sampling_config_.max_timestamp_offset <
    transform_sampling_config_.min_timestamp_offset) {
    throw std::invalid_argument(
      "max_timestamp_offset must be greater than or equal to min_timestamp_offset");
  }

  // subscribers
  map_sub_ = create_subscription<autoware_map_msgs::msg::LaneletMapBin>(
    "~/input/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&MapBasedDetector::map_callback, this, _1));
  camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    "~/input/camera_info", rclcpp::SensorDataQoS(),
    std::bind(&MapBasedDetector::camera_info_callback, this, _1));
  route_sub_ = create_subscription<autoware_planning_msgs::msg::LaneletRoute>(
    "~/input/route", rclcpp::QoS{1}.transient_local(),
    std::bind(&MapBasedDetector::route_callback, this, _1));

  // publishers
  roi_pub_ =
    this->create_publisher<tier4_perception_msgs::msg::TrafficLightRoiArray>("~/output/rois", 1);
  expect_roi_pub_ =
    this->create_publisher<tier4_perception_msgs::msg::TrafficLightRoiArray>("~/expect/rois", 1);
  viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/debug/markers", 1);
}

void MapBasedDetector::camera_info_callback(
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr input_msg)
{
  if (!detector_) {
    return;
  }

  const auto tf_map2camera_samples =
    sample_map_to_camera_transforms(tf_buffer_, input_msg->header, transform_sampling_config_);
  if (!tf_map2camera_samples) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "failed to get transform from map frame to camera frame");
    return;
  }

  auto result = detector_->detect(*tf_map2camera_samples, *input_msg);

  roi_pub_->publish(result.rough_rois);
  expect_roi_pub_->publish(result.expect_rois);
  viz_pub_->publish(result.markers);
}

void MapBasedDetector::map_callback(
  const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr input_msg)
{
  detector_ = std::make_unique<TrafficLightMapBasedDetector>(detector_config_, *input_msg);
}

void MapBasedDetector::route_callback(
  const autoware_planning_msgs::msg::LaneletRoute::ConstSharedPtr input_msg)
{
  if (!detector_) {
    RCLCPP_WARN(get_logger(), "failed to set traffic lights in route: map not received");
    return;
  }
  auto error = detector_->set_route(*input_msg);
  if (error) {
    RCLCPP_ERROR(get_logger(), "%s", error->message.c_str());
  }
}
}  // namespace autoware::traffic_light

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::traffic_light::MapBasedDetector)
