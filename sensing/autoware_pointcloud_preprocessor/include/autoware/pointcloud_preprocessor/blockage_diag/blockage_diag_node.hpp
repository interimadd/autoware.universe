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

#ifndef AUTOWARE__POINTCLOUD_PREPROCESSOR__BLOCKAGE_DIAG__BLOCKAGE_DIAG_NODE_HPP_
#define AUTOWARE__POINTCLOUD_PREPROCESSOR__BLOCKAGE_DIAG__BLOCKAGE_DIAG_NODE_HPP_

#include "autoware/pointcloud_preprocessor/blockage_diag/blockage_diag.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include <memory>
#include <optional>
#include <vector>

namespace autoware::pointcloud_preprocessor
{
using diagnostic_updater::DiagnosticStatusWrapper;
using diagnostic_updater::Updater;

class BlockageDiagComponent : public rclcpp::Node
{
private:
  /** \brief Parameter service callback result : needed to be hold */
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  /** \brief Parameter service callback */
  rcl_interfaces::msg::SetParametersResult param_callback(const std::vector<rclcpp::Parameter> & p);

  // ROS-specific components
  Updater updater_{this};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  image_transport::Publisher blockage_dust_merged_pub_;

  // Core blockage diagnosis engine (ROS-independent)
  std::unique_ptr<BlockageDiag> blockage_diag_;

  /**
   * @brief Main callback that processes incoming point cloud data.
   *
   * @param input The input point cloud message.
   */
  void update_diagnostics(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & input);

  /**
   * @brief Diagnostic updater callback for blockage validation.
   *
   * @param stat The diagnostic status wrapper to populate.
   */
  void run_blockage_check(DiagnosticStatusWrapper & stat) const;

  /**
   * @brief Diagnostic updater callback for dust validation.
   *
   * @param stat The diagnostic status wrapper to populate.
   */
  void run_dust_check(DiagnosticStatusWrapper & stat) const;

  /**
   * @brief Publish debug image if available.
   *
   * Image already has header set by BlockageDiag.
   *
   * @param debug_image The debug image to publish.
   */
  void publish_debug_images(const BlockageDiagDebugImages & debug_image);

public:
  explicit BlockageDiagComponent(const rclcpp::NodeOptions & options);
};

}  // namespace autoware::pointcloud_preprocessor

#endif  // AUTOWARE__POINTCLOUD_PREPROCESSOR__BLOCKAGE_DIAG__BLOCKAGE_DIAG_NODE_HPP_
