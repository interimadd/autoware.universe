// Copyright 2026 Tier IV, Inc.
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

#include "autoware/image_transport_decompressor/image_decompression.hpp"

#include <opencv2/opencv.hpp>

#include <sensor_msgs/image_encodings.hpp>

#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <vector>

// Unit test for decompress_image(). It is pure C++ (no rclcpp::init/shutdown), so all test data is
// built directly as sensor_msgs::msg::CompressedImage / cv::Mat values.

namespace
{
using autoware::image_transport_decompressor::decompress_image;
using sensor_msgs::msg::CompressedImage;

// Builds a real JPEG-encoded CompressedImage from a solid-color BGR image, so decompress_image()
// exercises the actual cv::imdecode() path rather than a hand-crafted byte string.
CompressedImage make_compressed_image(const std::string & format, const cv::Vec3b & bgr_color)
{
  constexpr int width = 8;
  constexpr int height = 8;
  cv::Mat bgr_image(height, width, CV_8UC3, bgr_color);

  std::vector<uint8_t> jpeg_bytes;
  // Lossless-ish encoding so the round-tripped pixel values stay close to the original color.
  const std::vector<int> jpeg_quality_100 = {cv::IMWRITE_JPEG_QUALITY, 100};
  cv::imencode(".jpg", bgr_image, jpeg_bytes, jpeg_quality_100);

  CompressedImage compressed_image;
  compressed_image.header.frame_id = "camera";
  compressed_image.format = format;
  compressed_image.data = jpeg_bytes;
  return compressed_image;
}

}  // namespace

TEST(ImageDecompressionTest, InvalidDataReturnsError)
{
  // Arrange
  CompressedImage compressed_image;
  compressed_image.format = "bgr8; jpeg compressed bgr8";
  compressed_image.data = {0x00, 0x01, 0x02};  // not a valid JPEG byte stream

  // Act
  const auto result = decompress_image(compressed_image, "default");

  // Assert
  ASSERT_FALSE(result.has_value());
}

TEST(ImageDecompressionTest, FormatWithoutSemicolonAlwaysDecodesAsBgr8)
{
  // Arrange
  // camera6/camera7 record `format: "jpeg"` (no ';' separator, see plan §7.0.2). In this case the
  // encoding is derived solely from the channel count, and the `encoding` parameter is ignored.
  const CompressedImage compressed_image = make_compressed_image("jpeg", cv::Vec3b(10, 20, 30));

  // Act
  const auto result = decompress_image(compressed_image, "rgb8");

  // Assert
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->encoding, sensor_msgs::image_encodings::BGR8);
}

TEST(ImageDecompressionTest, FormatWithSemicolonAndDefaultEncodingKeepsSourceEncoding)
{
  // Arrange
  const CompressedImage compressed_image =
    make_compressed_image("bgr8; jpeg compressed bgr8", cv::Vec3b(10, 20, 30));

  // Act
  const auto result = decompress_image(compressed_image, "default");

  // Assert
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->encoding, sensor_msgs::image_encodings::BGR8);
}

TEST(ImageDecompressionTest, FormatWithSemicolonAndRgb8EncodingConvertsColorOrder)
{
  // Arrange
  const cv::Vec3b bgr_color(10, 20, 200);  // blue=10, green=20, red=200
  const CompressedImage compressed_image =
    make_compressed_image("bgr8; jpeg compressed bgr8", bgr_color);

  // Act
  const auto result = decompress_image(compressed_image, "rgb8");

  // Assert
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->encoding, sensor_msgs::image_encodings::RGB8);
  ASSERT_EQ(result->data.size(), result->height * result->step);
  // The first pixel's channel order is reversed (rgb) relative to the source (bgr).
  const auto & pixel_bytes = result->data;
  EXPECT_NEAR(pixel_bytes[0], bgr_color[2], 5);  // red
  EXPECT_NEAR(pixel_bytes[2], bgr_color[0], 5);  // blue
}

TEST(ImageDecompressionTest, HeaderIsCopiedFromInput)
{
  // Arrange
  CompressedImage compressed_image = make_compressed_image("jpeg", cv::Vec3b(10, 20, 30));
  compressed_image.header.frame_id = "camera6/camera_optical_link";

  // Act
  const auto result = decompress_image(compressed_image, "default");

  // Assert
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->header.frame_id, "camera6/camera_optical_link");
}
