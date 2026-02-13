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
#include "autoware/pointcloud_preprocessor/blockage_diag/multi_frame_detection_aggregator.hpp"

#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <gtest/gtest.h>

namespace autoware::pointcloud_preprocessor
{

TEST(PointCloudValidationTest, MissingChannelFieldTest)
{
  sensor_msgs::msg::PointCloud2 cloud_without_channel;
  sensor_msgs::PointCloud2Modifier modifier(cloud_without_channel);
  modifier.setPointCloud2Fields(
    2, "azimuth", 1, sensor_msgs::msg::PointField::FLOAT32, "distance", 1,
    sensor_msgs::msg::PointField::FLOAT32);

  EXPECT_THROW({ validate_pointcloud_fields(cloud_without_channel); }, std::runtime_error);
}

TEST(PointCloudValidationTest, MissingAzimuthFieldTest)
{
  sensor_msgs::msg::PointCloud2 cloud_without_azimuth;
  sensor_msgs::PointCloud2Modifier modifier(cloud_without_azimuth);
  modifier.setPointCloud2Fields(
    2, "channel", 1, sensor_msgs::msg::PointField::UINT16, "distance", 1,
    sensor_msgs::msg::PointField::FLOAT32);

  EXPECT_THROW({ validate_pointcloud_fields(cloud_without_azimuth); }, std::runtime_error);
}

TEST(PointCloudValidationTest, MissingDistanceFieldTest)
{
  sensor_msgs::msg::PointCloud2 cloud_without_distance;
  sensor_msgs::PointCloud2Modifier modifier(cloud_without_distance);
  modifier.setPointCloud2Fields(
    2, "channel", 1, sensor_msgs::msg::PointField::UINT16, "azimuth", 1,
    sensor_msgs::msg::PointField::FLOAT32);

  EXPECT_THROW({ validate_pointcloud_fields(cloud_without_distance); }, std::runtime_error);
}

TEST(PointCloudValidationTest, ValidFieldsTest)
{
  sensor_msgs::msg::PointCloud2 cloud_with_all_fields;
  sensor_msgs::PointCloud2Modifier modifier(cloud_with_all_fields);
  modifier.setPointCloud2Fields(
    3, "channel", 1, sensor_msgs::msg::PointField::UINT16, "azimuth", 1,
    sensor_msgs::msg::PointField::FLOAT32, "distance", 1, sensor_msgs::msg::PointField::FLOAT32);

  EXPECT_NO_THROW({ validate_pointcloud_fields(cloud_with_all_fields); });
}

bool is_same_image(const cv::Mat & img1, const cv::Mat & img2)
{
  if (img1.size() != img2.size() || img1.type() != img2.type()) {
    return false;
  }
  cv::Mat diff;
  cv::compare(img1, img2, diff, cv::CMP_NE);
  return cv::countNonZero(diff) == 0;
}

TEST(MultiFrameDetectionAggregatorTest, ZeroBufferingIntervalReturnSameMask)
{
  // Setup aggregator with zero buffering interval
  MultiFrameDetectionAggregatorConfig config;
  config.buffering_frames = 4;
  config.buffering_interval = 0;
  MultiFrameDetectionAggregator aggregator(config);
  cv::Mat input_mask(10, 10, CV_8UC1, cv::Scalar(255));

  // Update aggregator and get result
  cv::Mat result = aggregator.update(input_mask);

  // Verify that the result matches the input mask
  EXPECT_TRUE(is_same_image(input_mask, result));
}

TEST(MultiFrameDetectionAggregatorTest, AllPixelsConsistentTest)
{
  MultiFrameDetectionAggregatorConfig config;
  config.buffering_frames = 4;
  config.buffering_interval = 1;
  MultiFrameDetectionAggregator aggregator(config);
  cv::Mat consistent_mask(10, 10, CV_8UC1, cv::Scalar(255));
  int total_pixels = consistent_mask.rows * consistent_mask.cols;

  // Update with same mask 4 times
  aggregator.update(consistent_mask);
  aggregator.update(consistent_mask);
  aggregator.update(consistent_mask);
  cv::Mat result = aggregator.update(consistent_mask);

  // All pixels should be detected
  EXPECT_EQ(cv::countNonZero(result), total_pixels);
}

// Helper function to create test configuration
BlockageDiagConfig create_test_config(
  bool enable_dust = false, bool enable_debug = false)
{
  BlockageDiagConfig config;

  // Depth converter config
  config.depth_converter_config.horizontal.angle_range_min_deg = -180.0;
  config.depth_converter_config.horizontal.angle_range_max_deg = 180.0;
  config.depth_converter_config.horizontal.horizontal_resolution = 0.4;
  config.depth_converter_config.vertical.vertical_bins = 128;
  config.depth_converter_config.vertical.is_channel_order_top2down = true;
  config.depth_converter_config.max_distance_range = 200.0;

  // Blockage detection config
  config.blockage_config.blockage_ratio_threshold = 0.5f;
  config.blockage_config.blockage_count_threshold = 5;
  config.blockage_config.blockage_kernel = 3;
  config.blockage_config.horizontal_ring_id = 64;
  config.blockage_config.horizontal_resolution = 0.4;
  config.blockage_config.angle_range_min_deg = -180.0;
  config.blockage_config.angle_range_max_deg = 180.0;

  // Blockage aggregator config
  config.blockage_aggregator_config.buffering_frames = 3;
  config.blockage_aggregator_config.buffering_interval = 1;

  // Dust config
  config.enable_dust_detection = enable_dust;
  if (enable_dust) {
    config.dust_config.dust_ratio_threshold = 0.5f;
    config.dust_config.dust_count_threshold = 5;
    config.dust_config.dust_kernel_size = 3;
    config.dust_config.horizontal_ring_id = 64;
    config.dust_aggregator_config.buffering_frames = 3;
    config.dust_aggregator_config.buffering_interval = 1;
  }

  // Debug config
  config.enable_debug_output = enable_debug;

  return config;
}

// Helper function to create test point cloud
sensor_msgs::msg::PointCloud2 create_test_pointcloud()
{
  sensor_msgs::msg::PointCloud2 cloud;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(
    3, "channel", 1, sensor_msgs::msg::PointField::UINT16, "azimuth", 1,
    sensor_msgs::msg::PointField::FLOAT32, "distance", 1, sensor_msgs::msg::PointField::FLOAT32);

  // Create a simple point cloud with 100 points
  modifier.resize(100);

  sensor_msgs::PointCloud2Iterator<uint16_t> iter_channel(cloud, "channel");
  sensor_msgs::PointCloud2Iterator<float> iter_azimuth(cloud, "azimuth");
  sensor_msgs::PointCloud2Iterator<float> iter_distance(cloud, "distance");

  for (size_t i = 0; i < 100; ++i, ++iter_channel, ++iter_azimuth, ++iter_distance) {
    *iter_channel = static_cast<uint16_t>(i % 128);  // Channel in range [0, 127]
    *iter_azimuth = static_cast<float>(i * 0.1);  // Azimuth in radians
    *iter_distance = 10.0f + static_cast<float>(i) * 0.5f;  // Distance in meters
  }

  return cloud;
}

TEST(BlockageDiagTest, ConstructorInitializesComponents)
{
  BlockageDiagConfig config = create_test_config();
  EXPECT_NO_THROW({ BlockageDiag diag(config); });
}

TEST(BlockageDiagTest, BasicBlockageDetectionNoDustNoDebug)
{
  // Arrange: Create config with blockage enabled, dust and debug disabled
  BlockageDiagConfig config = create_test_config(false, false);
  BlockageDiag diag(config);

  sensor_msgs::msg::PointCloud2 input = create_test_pointcloud();

  // Act
  BlockageDiagResult result = diag.update(input);

  // Assert
  EXPECT_FALSE(result.dust_diagnostic.has_value());
  EXPECT_FALSE(result.debug_images.has_value());
  EXPECT_FALSE(result.blockage_diagnostic.message.empty());
}

TEST(BlockageDiagTest, DustDetectionEnabled)
{
  // Arrange
  BlockageDiagConfig config = create_test_config(true, false);
  BlockageDiag diag(config);

  sensor_msgs::msg::PointCloud2 input = create_test_pointcloud();

  // Act
  BlockageDiagResult result = diag.update(input);

  // Assert
  EXPECT_TRUE(result.dust_diagnostic.has_value());
  EXPECT_FALSE(result.dust_diagnostic->message.empty());
  EXPECT_FALSE(result.debug_images.has_value());
}

TEST(BlockageDiagTest, DebugImagesGeneratedWithoutDust)
{
  // Arrange
  BlockageDiagConfig config = create_test_config(false, true);
  BlockageDiag diag(config);

  sensor_msgs::msg::PointCloud2 input = create_test_pointcloud();

  // Act
  BlockageDiagResult result = diag.update(input);

  // Assert
  ASSERT_TRUE(result.debug_images.has_value());
  EXPECT_FALSE(result.debug_images->blockage_mask_single_frame.empty());
  EXPECT_FALSE(result.debug_images->blockage_mask_multi_frame.empty());
  // Dust masks should be empty when dust detection is disabled
  EXPECT_TRUE(result.debug_images->dust_mask_single_frame.empty());
  EXPECT_TRUE(result.debug_images->dust_mask_multi_frame.empty());
}

TEST(BlockageDiagTest, DebugImagesGeneratedWithDust)
{
  // Arrange
  BlockageDiagConfig config = create_test_config(true, true);
  BlockageDiag diag(config);

  sensor_msgs::msg::PointCloud2 input = create_test_pointcloud();

  // Act
  BlockageDiagResult result = diag.update(input);

  // Assert
  ASSERT_TRUE(result.debug_images.has_value());
  EXPECT_FALSE(result.debug_images->blockage_mask_single_frame.empty());
  EXPECT_FALSE(result.debug_images->blockage_mask_multi_frame.empty());
  EXPECT_FALSE(result.debug_images->dust_mask_single_frame.empty());
  EXPECT_FALSE(result.debug_images->dust_mask_multi_frame.empty());
  EXPECT_FALSE(result.debug_images->blockage_dust_merged.empty());
  // Check that merged image is RGB
  EXPECT_EQ(result.debug_images->blockage_dust_merged.type(), CV_8UC3);
}

TEST(BlockageDiagTest, InvalidPointCloudThrowsException)
{
  // Arrange
  BlockageDiagConfig config = create_test_config();
  BlockageDiag diag(config);

  sensor_msgs::msg::PointCloud2 invalid_input;  // Missing required fields

  // Act & Assert
  EXPECT_THROW({ diag.update(invalid_input); }, std::runtime_error);
}

TEST(BlockageDiagTest, HeaderIsPreserved)
{
  // Arrange
  BlockageDiagConfig config = create_test_config();
  BlockageDiag diag(config);

  sensor_msgs::msg::PointCloud2 input = create_test_pointcloud();
  input.header.frame_id = "lidar_frame";
  input.header.stamp.sec = 12345;
  input.header.stamp.nanosec = 67890;

  // Act
  BlockageDiagResult result = diag.update(input);

  // Assert
  EXPECT_EQ(result.header.frame_id, "lidar_frame");
  EXPECT_EQ(result.header.stamp.sec, 12345);
  EXPECT_EQ(result.header.stamp.nanosec, 67890);
}

TEST(BlockageDiagTest, ConfigQueryMethods)
{
  // Test with dust disabled
  BlockageDiagConfig config1 = create_test_config(false, false);
  BlockageDiag diag1(config1);
  EXPECT_FALSE(diag1.is_dust_detection_enabled());
  EXPECT_FALSE(diag1.is_debug_output_enabled());

  // Test with dust and debug enabled
  BlockageDiagConfig config2 = create_test_config(true, true);
  BlockageDiag diag2(config2);
  EXPECT_TRUE(diag2.is_dust_detection_enabled());
  EXPECT_TRUE(diag2.is_debug_output_enabled());
}

TEST(BlockageDiagTest, MultiFrameProcessing)
{
  // Arrange
  BlockageDiagConfig config = create_test_config(false, true);
  BlockageDiag diag(config);

  // Act: Send multiple frames
  for (int i = 0; i < 5; ++i) {
    sensor_msgs::msg::PointCloud2 input = create_test_pointcloud();
    input.header.stamp.sec = i;
    BlockageDiagResult result = diag.update(input);

    // Assert: Each frame produces valid results
    ASSERT_TRUE(result.debug_images.has_value());
    EXPECT_FALSE(result.debug_images->blockage_mask_multi_frame.empty());
    EXPECT_EQ(result.header.stamp.sec, i);
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

}  // namespace autoware::pointcloud_preprocessor
