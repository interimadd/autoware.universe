// Copyright 2024 TIER IV, Inc.
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

#include "autoware/pointcloud_preprocessor/blockage_diag/blockage_diag_node.hpp"

#include "autoware/pointcloud_preprocessor/blockage_diag/blockage_diag.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace autoware::pointcloud_preprocessor
{
using diagnostic_msgs::msg::DiagnosticStatus;

BlockageDiagComponent::BlockageDiagComponent(const rclcpp::NodeOptions & options)
: rclcpp::Node("BlockageDiag", rclcpp::NodeOptions(options).start_parameter_services(false))
{
  // Build BlockageDiagConfig from ROS parameters
  BlockageDiagConfig config;

  // LiDAR configuration
  std::vector<double> angle_range_deg = declare_parameter<std::vector<double>>("angle_range");
  bool is_channel_order_top2down = declare_parameter<bool>("is_channel_order_top2down");
  int vertical_bins = declare_parameter<int>("vertical_bins");
  double horizontal_resolution = declare_parameter<double>("horizontal_resolution");
  double max_distance_range = declare_parameter<double>("max_distance_range");
  int horizontal_ring_id = declare_parameter<int>("horizontal_ring_id");

  // Validate parameters
  if (vertical_bins <= horizontal_ring_id) {
    RCLCPP_ERROR(
      this->get_logger(),
      "The horizontal_ring_id should be smaller than vertical_bins. Skip blockage diag!");
    return;
  }

  // Depth converter config
  config.depth_converter_config.horizontal.angle_range_min_deg = angle_range_deg[0];
  config.depth_converter_config.horizontal.angle_range_max_deg = angle_range_deg[1];
  config.depth_converter_config.horizontal.horizontal_resolution = horizontal_resolution;
  config.depth_converter_config.vertical.vertical_bins = vertical_bins;
  config.depth_converter_config.vertical.is_channel_order_top2down = is_channel_order_top2down;
  config.depth_converter_config.max_distance_range = max_distance_range;

  // Blockage detection config
  config.blockage_config.blockage_ratio_threshold =
    declare_parameter<float>("blockage_ratio_threshold");
  config.blockage_config.blockage_count_threshold =
    declare_parameter<int>("blockage_count_threshold");
  config.blockage_config.blockage_kernel = declare_parameter<int>("blockage_kernel");
  config.blockage_config.horizontal_ring_id = horizontal_ring_id;
  config.blockage_config.horizontal_resolution = horizontal_resolution;
  config.blockage_config.angle_range_min_deg = angle_range_deg[0];
  config.blockage_config.angle_range_max_deg = angle_range_deg[1];

  // Blockage aggregator config
  config.blockage_aggregator_config.buffering_frames =
    declare_parameter<int>("blockage_buffering_frames");
  config.blockage_aggregator_config.buffering_interval =
    declare_parameter<int>("blockage_buffering_interval");

  // Dust detection config
  config.enable_dust_detection = declare_parameter<bool>("enable_dust_diag");
  if (config.enable_dust_detection) {
    config.dust_config.dust_ratio_threshold = declare_parameter<float>("dust_ratio_threshold");
    config.dust_config.dust_count_threshold = declare_parameter<int>("dust_count_threshold");
    config.dust_config.dust_kernel_size = declare_parameter<int>("dust_kernel_size");
    config.dust_config.horizontal_ring_id = horizontal_ring_id;

    config.dust_aggregator_config.buffering_frames =
      declare_parameter<int>("dust_buffering_frames");
    config.dust_aggregator_config.buffering_interval =
      declare_parameter<int>("dust_buffering_interval");
  }

  // Debug config
  config.enable_debug_output = declare_parameter<bool>("publish_debug_image");

  // Create the unified blockage diagnostic engine
  blockage_diag_ = std::make_unique<BlockageDiag>(config);

  // Setup ROS-specific components
  if (config.enable_dust_detection && config.enable_debug_output) {
    blockage_dust_merged_pub_ =
      image_transport::create_publisher(this, "blockage_diag/debug/blockage_dust_merged_image");
  }

  pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "input", rclcpp::SensorDataQoS(),
    std::bind(&BlockageDiagComponent::update_diagnostics, this, std::placeholders::_1));

  updater_.setHardwareID("blockage_diag");
  updater_.add(
    std::string(this->get_namespace()) + ": blockage_validation",
    [this](auto & stat) { run_blockage_check(stat); });

  if (config.enable_dust_detection) {
    updater_.add(
      std::string(this->get_namespace()) + ": dust_validation",
      [this](auto & stat) { run_dust_check(stat); });
  }

  updater_.setPeriod(0.1);
}

void BlockageDiagComponent::run_blockage_check(DiagnosticStatusWrapper & stat) const
{
  if (!latest_result_.has_value()) {
    stat.summary(DiagnosticStatus::STALE, "No data received");
    return;
  }

  const auto & output = latest_result_->blockage_diagnostic;
  stat.summary(static_cast<unsigned char>(output.level), output.message);
  for (const auto & data : output.additional_data) {
    stat.add(data.key, data.value);
  }
}

void BlockageDiagComponent::run_dust_check(diagnostic_updater::DiagnosticStatusWrapper & stat) const
{
  if (!latest_result_.has_value() || !latest_result_->dust_diagnostic.has_value()) {
    stat.summary(DiagnosticStatus::STALE, "No dust data available");
    return;
  }

  const auto & output = latest_result_->dust_diagnostic.value();
  stat.summary(static_cast<unsigned char>(output.level), output.message);
  for (const auto & data : output.additional_data) {
    stat.add(data.key, data.value);
  }
}

void BlockageDiagComponent::publish_debug_images(const BlockageDiagDebugImages & debug_images)
{
  if (!debug_images) {
    return;
  }
  // blockage_dust_merged is the main debug visualization (header already set)
  if (!debug_images.blockage_dust_merged.data.empty()) {
    auto msg = std::make_shared<sensor_msgs::msg::Image>(debug_images.blockage_dust_merged);
    blockage_dust_merged_pub_.publish(msg);
  }
}

void BlockageDiagComponent::update_diagnostics(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & input)
{
  try {
    latest_result_ = blockage_diag_->update(*input);
    publish_debug_images(latest_result_->debug_images.value());
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR(get_logger(), "Blockage diagnostics failed: %s", e.what());
    latest_result_.reset();
  }
}
}  // namespace autoware::pointcloud_preprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::pointcloud_preprocessor::BlockageDiagComponent)
