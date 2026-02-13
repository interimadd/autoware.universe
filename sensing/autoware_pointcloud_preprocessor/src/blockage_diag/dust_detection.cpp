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

#include "autoware/pointcloud_preprocessor/blockage_diag/dust_detection.hpp"

#include <opencv2/imgproc.hpp>

#include <string>
#include <utility>

namespace autoware::pointcloud_preprocessor
{

DustDetector::DustDetector(const DustDetectionConfig & config) : config_(config)
{
}

DustDetectionResult DustDetector::compute_dust_diagnostics(const cv::Mat & depth_image_16u)
{
  assert(depth_image_16u.type() == CV_16UC1);
  auto dimensions = depth_image_16u.size();

  // Quantize 16-bit to 8-bit (downscale by 256 to prevent overflow)
  cv::Mat depth_image_8u(dimensions, CV_8UC1, cv::Scalar(0));
  depth_image_16u.convertTo(depth_image_8u, CV_8UC1, 1.0 / 256);

  // Create no-return mask (values 0-1 are considered no-return)
  cv::Mat no_return_mask(dimensions, CV_8UC1, cv::Scalar(0));
  cv::inRange(depth_image_8u, 0, 1, no_return_mask);

  // Segment into ground and sky
  cv::Mat sky_blank;
  no_return_mask(cv::Rect(0, 0, dimensions.width, config_.horizontal_ring_id)).copyTo(sky_blank);

  cv::Mat single_dust_ground_img;
  no_return_mask(cv::Rect(0, config_.horizontal_ring_id, dimensions.width, dimensions.height - config_.horizontal_ring_id))
    .copyTo(single_dust_ground_img);

  // It is normal for the sky region to be blank, therefore ignore it.
  sky_blank.setTo(cv::Scalar(0));

  int kernel_size = 2 * config_.dust_kernel_size + 1;
  int kernel_center = config_.dust_kernel_size;
  cv::Mat kernel = cv::getStructuringElement(
    cv::MORPH_RECT, cv::Size(kernel_size, kernel_size), cv::Point(kernel_center, kernel_center));

  cv::dilate(single_dust_ground_img, single_dust_ground_img, kernel);
  cv::erode(single_dust_ground_img, single_dust_ground_img, kernel);
  cv::inRange(single_dust_ground_img, 254, 255, single_dust_ground_img);

  // Re-assemble the processed ground dust image and the sky blank.
  cv::Mat single_dust_img(dimensions, CV_8UC1, cv::Scalar(0));
  cv::vconcat(sky_blank, single_dust_ground_img, single_dust_img);

  result_.ground_dust_ratio = static_cast<float>(cv::countNonZero(single_dust_ground_img)) /
                              (single_dust_ground_img.cols * single_dust_ground_img.rows);

  if (result_.ground_dust_ratio > config_.dust_ratio_threshold) {
    if (result_.dust_frame_count < 2 * config_.dust_count_threshold) {
      result_.dust_frame_count++;
    }
  } else {
    result_.dust_frame_count = 0;
  }

  result_.dust_mask = single_dust_img;

  return result_;
}

DiagnosticOutput DustDetector::get_dust_diagnostics_output() const
{
  DiagnosticOutput output;

  output.additional_data.push_back(
    {"ground_dust_ratio", std::to_string(result_.ground_dust_ratio)});

  output.level = DiagnosticLevel::OK;
  output.message = "OK";

  if (result_.ground_dust_ratio < 0.0f) {
    output.level = DiagnosticLevel::STALE;
    output.message = "STALE";
  } else if (
    (result_.ground_dust_ratio > config_.dust_ratio_threshold) &&
    (result_.dust_frame_count > config_.dust_count_threshold)) {
    output.level = DiagnosticLevel::ERROR;
    output.message = "ERROR";
  } else if (result_.ground_dust_ratio > 0.0f) {
    output.level = DiagnosticLevel::WARN;
    output.message = "WARN";
  }

  if (result_.ground_dust_ratio > 0.0f) {
    output.message = output.message + ": LIDAR ground dust";
  }
  return output;
}

}  // namespace autoware::pointcloud_preprocessor
