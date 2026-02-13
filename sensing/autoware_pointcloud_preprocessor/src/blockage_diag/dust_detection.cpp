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

namespace
{

/**
 * @brief Quantize a 16-bit image to 8-bit.
 *
 * The values are scaled by `1.0 / 256` to prevent overflow.
 *
 * @param image_16u The input 16-bit image.
 * @return cv::Mat The quantized 8-bit image. The data type is `CV_8UC1`.
 */
cv::Mat quantize_to_8u(const cv::Mat & image_16u)
{
  assert(image_16u.type() == CV_16UC1);
  auto dimensions = image_16u.size();

  cv::Mat image_8u(dimensions, CV_8UC1, cv::Scalar(0));
  // UINT16_MAX = 65535, UINT8_MAX = 255, so downscale by ceil(65535 / 255) = 256.
  image_16u.convertTo(image_8u, CV_8UC1, 1.0 / 256);
  return image_8u;
}

/**
 * @brief Make a no-return mask from the input depth image.
 *
 * The mask is a binary image where 255 is no-return and 0 is return.
 *
 * @param depth_image The input depth image.
 * @return cv::Mat The no-return mask. The data type is `CV_8UC1`.
 */
cv::Mat make_no_return_mask(const cv::Mat & depth_image)
{
  assert(depth_image.type() == CV_8UC1);
  auto dimensions = depth_image.size();

  cv::Mat no_return_mask(dimensions, CV_8UC1, cv::Scalar(0));
  cv::inRange(depth_image, 0, 1, no_return_mask);

  return no_return_mask;
}

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

}  // namespace

DustDetector::DustDetector(const DustDetectionConfig & config) : config_(config)
{
}

DustDetectionResult DustDetector::compute_dust_diagnostics(const cv::Mat & depth_image_16u)
{
  cv::Mat depth_image_8u = quantize_to_8u(depth_image_16u);
  cv::Mat no_return_mask = make_no_return_mask(depth_image_8u);

  assert(no_return_mask.type() == CV_8UC1);
  auto dimensions = no_return_mask.size();

  auto [single_dust_ground_img, sky_blank] =
    segment_into_ground_and_sky(no_return_mask, config_.horizontal_ring_id);

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
