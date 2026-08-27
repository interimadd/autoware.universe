// Copyright 2026 TIER IV, Inc.
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

#ifndef TRAFFIC_LIGHT_RECOGNITION_NODE_HPP_
#define TRAFFIC_LIGHT_RECOGNITION_NODE_HPP_

#include <autoware/traffic_light_recognition/traffic_light_recognition.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tier4_perception_msgs/msg/traffic_light_array.hpp>
#include <tier4_perception_msgs/msg/traffic_light_roi_array.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <optional>

namespace autoware::traffic_light
{

// Declares this package's ROS 2 parameters on `node` and returns the resulting (flat)
// TrafficLightRecognitionConfig the ROS-free TrafficLightRecognition core consumes. Declaring
// parameters is a Node concern -- the core itself never touches rclcpp -- but this is a plain
// read of the node's parameter tree and nothing else: every fixed (non-parameter) value the
// underlying cores need (precision, mean/std, gpu_id, classify_traffic_light_type, the
// map_based_detector calibration-error margins and range/angle cutoffs, ...) is filled in by
// TrafficLightRecognition's constructor / build_engines() (traffic_light_recognition.cpp), not
// here. Exposed as a free function (rather than kept file-local to the .cpp) so
// test_traffic_light_recognition_params.cpp can exercise it directly, without a full Node.
//
// model_path / label_path / roi_remap_path (per detector/classifier) are declared here as plain
// top-level parameters, not part of the versioned config file (plan §5.1), so a launch file can
// inject them without the config file ever hard-coding a $HOME/autoware_data path.
TrafficLightRecognitionConfig declare_config(rclcpp::Node * node);

// Node adapter around TrafficLightRecognition (plan §4). Composes, for one camera, everything
// production's traffic_light_node_container.launch.py currently spreads across 5 Nodes
// (map_based_detector, whole_image yolox detector, selector, car/pedestrian classifiers, category
// merger) into a single Node with a single message_filters sync on (Image, CameraInfo).
class TrafficLightRecognitionNode : public rclcpp::Node
{
public:
  explicit TrafficLightRecognitionNode(const rclcpp::NodeOptions & node_options);

private:
  using SyncPolicy = message_filters::sync_policies::ExactTime<
    sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>;
  using Sync = message_filters::Synchronizer<SyncPolicy>;

  void vector_map_callback(const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr msg);
  void route_callback(const autoware_planning_msgs::msg::LaneletRoute::ConstSharedPtr msg);
  void sync_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg);

  // Builds detector/classifier-only cores from `config_` to force the TensorRT engines to build,
  // then shuts the node down. Used by build_only (plan §4.4): unlike the full
  // TrafficLightRecognition, this does not need the vector map.
  void build_engines_and_shutdown();

  TrafficLightRecognitionConfig config_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::unique_ptr<TrafficLightRecognition> recognition_;
  // A route received before the vector map is stashed here and applied once the map arrives
  // (plan §4.3 step 3).
  autoware_planning_msgs::msg::LaneletRoute::ConstSharedPtr pending_route_;

  rclcpp::Subscription<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr vector_map_sub_;
  rclcpp::Subscription<autoware_planning_msgs::msg::LaneletRoute>::SharedPtr route_sub_;

  message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> camera_info_sub_;
  std::unique_ptr<Sync> sync_;

  rclcpp::Publisher<tier4_perception_msgs::msg::TrafficLightArray>::SharedPtr signals_pub_;
  rclcpp::Publisher<tier4_perception_msgs::msg::TrafficLightRoiArray>::SharedPtr rois_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
};

}  // namespace autoware::traffic_light

#endif  // TRAFFIC_LIGHT_RECOGNITION_NODE_HPP_
