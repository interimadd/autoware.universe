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

#ifndef AUTOWARE__POINTCLOUD_PREPROCESSOR__BLOCKAGE_DIAG__BLOCKAGE_DIAG_HPP_
#define AUTOWARE__POINTCLOUD_PREPROCESSOR__BLOCKAGE_DIAG__BLOCKAGE_DIAG_HPP_

#include "blockage_diag_types.hpp"

#include "autoware/pointcloud_preprocessor/blockage_diag/blockage_detection.hpp"
#include "autoware/pointcloud_preprocessor/blockage_diag/dust_detection.hpp"
#include "autoware/pointcloud_preprocessor/blockage_diag/multi_frame_detection_aggregator.hpp"
#include "autoware/pointcloud_preprocessor/blockage_diag/pointcloud2_to_depth_image.hpp"

#include <opencv2/core/mat.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

#include <optional>
#include <string>
#include <utility>
#include <vector>

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
 * @brief Debug image generated during blockage/dust detection.
 *
 * This is an alias for the merged blockage/dust visualization image.
 */
using BlockageDiagDebugImages = sensor_msgs::msg::Image;

/**
 * @brief Result of blockage diagnosis.
 *
 * This structure contains optional debug image.
 */
struct BlockageDiagResult
{
  std::optional<sensor_msgs::msg::Image> debug_image;
  std_msgs::msg::Header header;
};

/**
 * @brief Quantize a 16-bit image to 8-bit.
 *
 * The values are scaled by `1.0 / 256` to prevent overflow.
 *
 * @param image_16u The input 16-bit image.
 * @return cv::Mat The quantized 8-bit image. The data type is `CV_8UC1`.
 */
cv::Mat quantize_to_8u(const cv::Mat & image_16u);

/**
 * @brief Make a no-return mask from the input depth image.
 *
 * The mask is a binary image where 255 is no-return and 0 is return.
 *
 * @param depth_image The input depth image.
 * @return cv::Mat The no-return mask. The data type is `CV_8UC1`.
 */
cv::Mat make_no_return_mask(const cv::Mat & depth_image);

/**
 * @brief Segments a given mask into two masks, according to the ground/sky segmentation
 * parameters.
 *
 * @param mask The input mask. The data type is `CV_8UC1`.
 * @param horizontal_ring_id The ring ID that separates ground and sky.
 * @return std::pair<cv::Mat, cv::Mat> The pair {ground_mask, sky_mask}. The data type is
 * `CV_8UC1`.
 */
std::pair<cv::Mat, cv::Mat> segment_into_ground_and_sky(
  const cv::Mat & mask, int horizontal_ring_id);

/**
 * @brief Validate that the PointCloud2 message has required fields for blockage diagnosis.
 *
 * @param input The input point cloud.
 * @throws std::runtime_error if any required field is missing.
 */
void validate_pointcloud_fields(const sensor_msgs::msg::PointCloud2 & input);

/**
 * @brief Facade class for blockage and dust detection.
 *
 * This class encapsulates all blockage/dust detection logic and provides a simple interface
 * that is independent of ROS node implementation. It coordinates depth image conversion,
 * blockage detection, dust detection, and multi-frame aggregation.
 */
class BlockageDiag
{
public:
  /**
   * @brief Constructor.
   *
   * @param config Configuration parameters for all detection components.
   */
  explicit BlockageDiag(const BlockageDiagConfig & config);

  /**
   * @brief Process a point cloud and perform blockage/dust diagnosis.
   *
   * This method validates the input, converts it to a depth image, runs detection algorithms,
   * and returns comprehensive diagnostic results.
   *
   * @param input The input point cloud message.
   * @return BlockageDiagResult containing diagnostics and optional debug images.
   * @throws std::runtime_error if point cloud validation fails.
   */
  BlockageDiagResult update(const sensor_msgs::msg::PointCloud2 & input);

  /**
   * @brief Get the latest blockage detection diagnostic result.
   *
   * @return DiagnosticOutput containing the blockage diagnostic. Returns STALE if no data available.
   */
  DiagnosticOutput get_blockage_detection_diag() const;

  /**
   * @brief Get the latest dust detection diagnostic result.
   *
   * @return DiagnosticOutput containing the dust diagnostic. Returns STALE if dust detection is disabled or no data available.
   */
  DiagnosticOutput get_dust_detection_diag() const;

  /**
   * @brief Get the latest debug image.
   *
   * @return BlockageDiagDebugImages (sensor_msgs::msg::Image) containing the merged debug visualization. Returns empty image if not available.
   */
  BlockageDiagDebugImages get_debug_images() const;

private:
  BlockageDiagConfig config_;

  // Cached latest diagnostic result
  std::optional<BlockageDiagResult> latest_result_;

  // Core components
  std::unique_ptr<pointcloud2_to_depth_image::PointCloud2ToDepthImage> depth_converter_;
  std::unique_ptr<BlockageDetector> blockage_detector_;
  std::unique_ptr<MultiFrameDetectionAggregator> blockage_aggregator_;

  // Optional dust detection components
  std::unique_ptr<DustDetector> dust_detector_;
  std::unique_ptr<MultiFrameDetectionAggregator> dust_aggregator_;

  /**
   * @brief Create debug image from detection results.
   *
   * @param blockage_mask_multi_frame Multi-frame aggregated blockage mask.
   * @param dust_result Single-frame dust detection result (optional).
   * @param dust_mask_multi_frame Multi-frame aggregated dust mask (optional).
   * @param header Header to set on the generated image.
   * @return BlockageDiagDebugImages (sensor_msgs::msg::Image) containing the merged debug visualization.
   */
  sensor_msgs::msg::Image create_debug_images(
    const cv::Mat & blockage_mask_multi_frame,
    const std::optional<DustDetectionResult> & dust_result,
    const std::optional<cv::Mat> & dust_mask_multi_frame,
    const std_msgs::msg::Header & header) const;
};

}  // namespace autoware::pointcloud_preprocessor

#endif  // AUTOWARE__POINTCLOUD_PREPROCESSOR__BLOCKAGE_DIAG__BLOCKAGE_DIAG_HPP_
