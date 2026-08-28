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

#ifndef TRAFFIC_LIGHT_FUSION__TRAFFIC_LIGHT_FUSION_NODE_HPP_
#define TRAFFIC_LIGHT_FUSION__TRAFFIC_LIGHT_FUSION_NODE_HPP_

#include "traffic_light_fusion.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tier4_perception_msgs/msg/traffic_light_array.hpp>
#include <tier4_perception_msgs/msg/traffic_light_roi_array.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>

#include <memory>
#include <vector>

namespace autoware::traffic_light
{

// Declares this Node's ROS 2 parameters on `node` and returns the resulting
// TrafficLightFusionConfig the ROS-free TrafficLightFusion core consumes. The parameter names are
// production's three back-end Nodes' own names, each moved under the prefix of the component it
// belongs to (`multi_camera_fusion.` / `arbiter.` / `crosswalk_estimator.`), since one Node now
// carries all three components' parameters and their flat names would otherwise collide.
// `source_priority` is normalized here exactly as TrafficLightArbiterNode normalizes it, so the
// core never has to guard against a typo. Exposed as a free function (rather than kept file-local
// to the .cpp) so test_traffic_light_fusion_params.cpp can exercise it without a full Node.
//
// `camera_namespaces` is not part of the config struct: it selects which cameras this Node
// subscribes to, which is Node I/O, not core configuration. It is read separately by the Node.
TrafficLightFusionConfig declare_fusion_config(rclcpp::Node * node);

// Node adapter around TrafficLightFusion. Composes everything production's
// traffic_light_node_container.launch.py currently spreads across 3 Nodes
// (multi_camera_fusion, arbiter, crosswalk_traffic_light_estimator) into a single Node with one
// message_filters sync on (CameraInfo, TrafficLightRoiArray, TrafficLightArray) per camera --
// the same per-camera sync MultiCameraFusionNode already does, since that sync's output is what
// drives the whole back-end chain.
class TrafficLightFusionNode : public rclcpp::Node
{
public:
  explicit TrafficLightFusionNode(const rclcpp::NodeOptions & node_options);

private:
  using CameraInfo = sensor_msgs::msg::CameraInfo;
  using RoiArray = tier4_perception_msgs::msg::TrafficLightRoiArray;
  using SignalArray = tier4_perception_msgs::msg::TrafficLightArray;
  using TrafficLightGroupArray = autoware_perception_msgs::msg::TrafficLightGroupArray;

  // ExactTime only: the three inputs of one camera are all produced from that camera's single
  // frame (by autoware_traffic_light_pipeline's own front-end Node, which republishes
  // camera_info unchanged alongside its results), so their stamps match exactly.
  // MultiCameraFusionNode's `approximate_sync` parameter is intentionally not carried over.
  using SyncPolicy = message_filters::sync_policies::ExactTime<CameraInfo, RoiArray, SignalArray>;
  using Sync = message_filters::Synchronizer<SyncPolicy>;

  // One camera's worth of subscriptions plus the synchronizer that ties them together. Held by
  // pointer because message_filters::Subscriber / Synchronizer are neither copyable nor movable.
  struct CameraSubscription
  {
    std::unique_ptr<message_filters::Subscriber<CameraInfo>> camera_info_sub;
    std::unique_ptr<message_filters::Subscriber<RoiArray>> rois_sub;
    std::unique_ptr<message_filters::Subscriber<SignalArray>> signals_sub;
    std::unique_ptr<Sync> sync;
  };

  void vector_map_callback(const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr msg);
  void sync_callback(
    const CameraInfo::ConstSharedPtr & camera_info_msg, const RoiArray::ConstSharedPtr & rois_msg,
    const SignalArray::ConstSharedPtr & signals_msg);

  TrafficLightFusionConfig config_;

  std::unique_ptr<TrafficLightFusion> fusion_;

  rclcpp::Subscription<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr vector_map_sub_;
  std::vector<CameraSubscription> camera_subscriptions_;

  rclcpp::Publisher<TrafficLightGroupArray>::SharedPtr traffic_signals_pub_;
};

}  // namespace autoware::traffic_light

#endif  // TRAFFIC_LIGHT_FUSION__TRAFFIC_LIGHT_FUSION_NODE_HPP_
