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

#include "autoware/pointcloud_preprocessor/blockage_diag/blockage_diag.hpp"

#include "autoware/pointcloud_preprocessor/blockage_diag/blockage_detection.hpp"
#include "autoware/pointcloud_preprocessor/blockage_diag/dust_detection.hpp"
#include "autoware/pointcloud_preprocessor/blockage_diag/multi_frame_detection_aggregator.hpp"
#include "autoware/pointcloud_preprocessor/blockage_diag/pointcloud2_to_depth_image.hpp"

#include <opencv2/imgproc.hpp>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <string>
#include <utility>
#include <vector>

namespace autoware::pointcloud_preprocessor
{

cv::Mat quantize_to_8u(const cv::Mat & image_16u)
{
  assert(image_16u.type() == CV_16UC1);
  auto dimensions = image_16u.size();

  cv::Mat image_8u(dimensions, CV_8UC1, cv::Scalar(0));
  // UINT16_MAX = 65535, UINT8_MAX = 255, so downscale by ceil(65535 / 255) = 256.
  image_16u.convertTo(image_8u, CV_8UC1, 1.0 / 256);
  return image_8u;
}

cv::Mat make_no_return_mask(const cv::Mat & depth_image)
{
  assert(depth_image.type() == CV_8UC1);
  auto dimensions = depth_image.size();

  cv::Mat no_return_mask(dimensions, CV_8UC1, cv::Scalar(0));
  cv::inRange(depth_image, 0, 1, no_return_mask);

  return no_return_mask;
}

std::pair<cv::Mat, cv::Mat> segment_into_ground_and_sky(
  const cv::Mat & mask, int horizontal_ring_id)
{
  assert(mask.type() == CV_8UC1);
  auto dimensions = mask.size();

  cv::Mat sky_mask;
  mask(cv::Rect(0, 0, dimensions.width, horizontal_ring_id)).copyTo(sky_mask);

  cv::Mat ground_mask;
  mask(cv::Rect(0, horizontal_ring_id, dimensions.width, dimensions.height - horizontal_ring_id))
    .copyTo(ground_mask);

  return {ground_mask, sky_mask};
}

void validate_pointcloud_fields(const sensor_msgs::msg::PointCloud2 & input)
{
  std::vector<std::string> required_fields = {"channel", "azimuth", "distance"};

  for (const auto & field : input.fields) {
    auto it = std::find(required_fields.begin(), required_fields.end(), field.name);
    if (it != required_fields.end()) {
      required_fields.erase(it);
    }
  }

  bool has_all_required_fields = required_fields.empty();
  if (has_all_required_fields) {
    return;
  }

  std::string error_msg = "PointCloud2 missing required fields:";
  for (const auto & missing_field : required_fields) {
    error_msg += " " + missing_field;
  }
  throw std::runtime_error(error_msg);
}

BlockageDiag::BlockageDiag(const BlockageDiagConfig & config) : config_(config)
{
  // Initialize depth converter
  depth_converter_ =
    std::make_unique<pointcloud2_to_depth_image::PointCloud2ToDepthImage>(
      config_.depth_converter_config);

  // Initialize blockage detector and aggregator
  blockage_detector_ = std::make_unique<BlockageDetector>(config_.blockage_config);
  blockage_aggregator_ =
    std::make_unique<MultiFrameDetectionAggregator>(config_.blockage_aggregator_config);

  // Initialize dust detector and aggregator if enabled
  if (config_.enable_dust_detection) {
    dust_detector_ = std::make_unique<DustDetector>(config_.dust_config);
    dust_aggregator_ =
      std::make_unique<MultiFrameDetectionAggregator>(config_.dust_aggregator_config);
  }
}

BlockageDiagResult BlockageDiag::update(const sensor_msgs::msg::PointCloud2 & input)
{
  // Validate input
  validate_pointcloud_fields(input);

  BlockageDiagResult result;
  result.header = input.header;

  // Convert to depth image
  cv::Mat depth_image_16u = depth_converter_->make_normalized_depth_image(input);

  // Blockage detection
  BlockageDetectionResult blockage_result =
    blockage_detector_->compute_blockage_diagnostics(depth_image_16u);
  cv::Mat blockage_mask_multi_frame = blockage_aggregator_->update(blockage_result.blockage_mask);

  // Dust detection (optional)
  std::optional<DustDetectionResult> dust_result;
  std::optional<cv::Mat> dust_mask_multi_frame;
  if (config_.enable_dust_detection && dust_detector_) {
    dust_result = dust_detector_->compute_dust_diagnostics(depth_image_16u);
    dust_mask_multi_frame = dust_aggregator_->update(dust_result->dust_mask);
  }

  // Generate debug image (optional)
  if (config_.enable_debug_output) {
    result.debug_image = create_debug_images(
      blockage_result, blockage_mask_multi_frame, dust_result, dust_mask_multi_frame, input.header);
  }

  // Cache the result for later retrieval
  latest_result_ = result;

  return result;
}

DiagnosticOutput BlockageDiag::get_blockage_detection_diag() const
{
  return blockage_detector_->get_blockage_diagnostics_output();
}

DiagnosticOutput BlockageDiag::get_dust_detection_diag() const
{
  DiagnosticOutput output;

  if (!config_.enable_dust_detection || !dust_detector_) {
    output.level = DiagnosticLevel::STALE;
    output.message = "Dust detection is disabled";
    return output;
  }

  output = dust_detector_->get_dust_diagnostics_output();
  return output;
}

BlockageDiagDebugImages BlockageDiag::get_debug_images() const
{
  if (!latest_result_.has_value() || !latest_result_->debug_image.has_value()) {
    return sensor_msgs::msg::Image();
  }
  return latest_result_->debug_image.value();
}

sensor_msgs::msg::Image BlockageDiag::create_debug_images(
  const BlockageDetectionResult & blockage_result, const cv::Mat & blockage_mask_multi_frame,
  const std::optional<DustDetectionResult> & dust_result,
  const std::optional<cv::Mat> & dust_mask_multi_frame, const std_msgs::msg::Header & header) const
{
  // Create merged visualization: red for blockage, yellow for dust (if enabled)
  auto dimensions = blockage_mask_multi_frame.size();
  cv::Mat merged_img(dimensions, CV_8UC3, cv::Scalar(0, 0, 0));
  merged_img.setTo(cv::Vec3b(0, 0, 255), blockage_mask_multi_frame);  // red: blockage

  if (dust_result.has_value() && dust_mask_multi_frame.has_value()) {
    merged_img.setTo(cv::Vec3b(0, 255, 255), dust_mask_multi_frame.value());  // yellow: dust
  }

  return *cv_bridge::CvImage(header, "bgr8", merged_img).toImageMsg();
}

}  // namespace autoware::pointcloud_preprocessor
