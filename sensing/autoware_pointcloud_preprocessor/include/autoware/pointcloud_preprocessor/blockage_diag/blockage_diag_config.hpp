// Copyright 2026 TIER IV
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

#ifndef AUTOWARE__POINTCLOUD_PREPROCESSOR__BLOCKAGE_DIAG__BLOCKAGE_DIAG_CONFIG_HPP_
#define AUTOWARE__POINTCLOUD_PREPROCESSOR__BLOCKAGE_DIAG__BLOCKAGE_DIAG_CONFIG_HPP_

// Include detection headers to get config structure definitions
// Note: These headers include blockage_diag.hpp for DiagnosticOutput, but that's OK
// because blockage_diag.hpp include guards prevent circular includes
#include "autoware/pointcloud_preprocessor/blockage_diag/blockage_detection.hpp"
#include "autoware/pointcloud_preprocessor/blockage_diag/dust_detection.hpp"
#include "autoware/pointcloud_preprocessor/blockage_diag/multi_frame_detection_aggregator.hpp"
#include "autoware/pointcloud_preprocessor/blockage_diag/pointcloud2_to_depth_image.hpp"

#include <opencv2/core/mat.hpp>

#include <std_msgs/msg/header.hpp>

#include <optional>

namespace autoware::pointcloud_preprocessor
{

/**
 * @brief Configuration for the BlockageDiag class.
 *
 * This structure consolidates all configuration parameters needed for blockage and dust detection.
 */
struct BlockageDiagConfig
{
  pointcloud2_to_depth_image::ConverterConfig depth_converter_config;
  BlockageDetectionConfig blockage_config;
  MultiFrameDetectionAggregatorConfig blockage_aggregator_config;
  bool enable_dust_detection;
  DustDetectionConfig dust_config;
  MultiFrameDetectionAggregatorConfig dust_aggregator_config;
  bool enable_debug_output;
};

/**
 * @brief Debug images generated during blockage/dust detection.
 *
 * This structure contains various masks and visualizations for debugging purposes.
 */
struct BlockageDiagDebugImages
{
  cv::Mat blockage_mask_single_frame;
  cv::Mat blockage_mask_multi_frame;
  cv::Mat dust_mask_single_frame;
  cv::Mat dust_mask_multi_frame;
  cv::Mat blockage_dust_merged;
};

/**
 * @brief Result of blockage diagnosis.
 *
 * This structure contains diagnostic outputs and optional debug images.
 */
struct BlockageDiagResult
{
  DiagnosticOutput blockage_diagnostic;
  std::optional<DiagnosticOutput> dust_diagnostic;
  std::optional<BlockageDiagDebugImages> debug_images;
  std_msgs::msg::Header header;
};

}  // namespace autoware::pointcloud_preprocessor

#endif  // AUTOWARE__POINTCLOUD_PREPROCESSOR__BLOCKAGE_DIAG__BLOCKAGE_DIAG_CONFIG_HPP_
