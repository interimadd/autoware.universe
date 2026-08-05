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

#ifndef AUTOWARE__IMAGE_TRANSPORT_DECOMPRESSOR__IMAGE_DECOMPRESSION_HPP_
#define AUTOWARE__IMAGE_TRANSPORT_DECOMPRESSOR__IMAGE_DECOMPRESSION_HPP_

#include <tl_expected/expected.hpp>

#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <string>

namespace autoware::image_transport_decompressor
{
/**
 * @brief decode a compressed image message into a raw image message
 * @param[in] compressed_image_msg input compressed image message (jpeg/png encoded)
 * @param[in] encoding requested output encoding ("rgb8" / "bgr8" / "default"). Only used when
 * `compressed_image_msg.format` carries the source encoding (i.e. contains a ';' separator, e.g.
 * "bgr8; jpeg compressed bgr8"); ignored for the older format without a ';' separator, in which
 * case the output encoding is determined solely by the decoded image's channel count.
 * @return decoded image message on success (header copied from the input), or an error message
 * describing why decoding failed (corrupt/undecodable data or an unsupported channel count)
 */
tl::expected<sensor_msgs::msg::Image, std::string> decompress_image(
  const sensor_msgs::msg::CompressedImage & compressed_image_msg, const std::string & encoding);

}  // namespace autoware::image_transport_decompressor

#endif  // AUTOWARE__IMAGE_TRANSPORT_DECOMPRESSOR__IMAGE_DECOMPRESSION_HPP_
