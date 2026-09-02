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

// Offline evaluation runner for the full pipeline: TrafficLightRecognition (front-end) followed
// by TrafficLightFusion (back-end), both this package's ROS-free cores.
//
// Given a t4dataset directory it reads every (image, camera_info) pair out of the dataset's
// rosbag, once, up front (load_frames()). It then runs the front-end over every frame in
// ascending (stamp, camera_index) order to completion (pass A), and only afterwards feeds that
// same-order result sequence through one TrafficLightFusion instance (pass B) -- rather than
// interleaving front-end and back-end calls frame by frame the way the Node graph (and
// component_test's run_traffic_light_pipeline) does. TrafficLightFusion is stateful but never
// reads the clock (see traffic_light_fusion.hpp), so replaying the same input sequence in the
// same order through a fresh instance in a second pass produces identical output to interleaving
// it live -- and keeping the two passes separate keeps this file's control flow simple. Finally
// every result (front-end and back-end) is written to an output rosbag under production topic
// names (pass C). No rclcpp::init, no executor, no DDS anywhere in this file.
//
// Reference implementation for the three-pass structure and the back-end config shape:
// autoware_component_test's run_traffic_light_pipeline_main.cpp.
//
// The front-end-only counterpart is run_traffic_light_recognition_evaluation, which shares this
// file's yaml/rosbag-loading plumbing via evaluation_common.{hpp,cpp}.
//
// Usage:
//   run_traffic_light_pipeline_evaluation
//     --config <evaluation config yaml, with a fusion: section>
//     --dataset <t4dataset dir>
//     --output-bag <output bag dir>
//
// The dataset layout is fixed (same convention as the Component Test harness):
//   <dataset>/input_bag
//   <dataset>/map/lanelet2_map.osm
//   <dataset>/map/map_projector_info.yaml

#include "evaluation_common.hpp"
#include "traffic_light_fusion/traffic_light_fusion.hpp"
#include "traffic_light_recognition/traffic_light_recognition.hpp"

#include <rclcpp/time.hpp>
#include <rosbag2_cpp/writer.hpp>

#include <sensor_msgs/msg/camera_info.hpp>

#include <yaml-cpp/yaml.h>

#include <cstddef>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
using autoware::traffic_light::TrafficLightFusion;
using autoware::traffic_light::TrafficLightFusionConfig;
using autoware::traffic_light::TrafficLightRecognition;
using autoware::traffic_light::TrafficLightRecognitionResult;
using autoware::traffic_light::evaluation::decode_frame_image;
using autoware::traffic_light::evaluation::EvaluationConfig;
using autoware::traffic_light::evaluation::Frame;
using autoware::traffic_light::evaluation::require;

// --- back-end config (yaml) -----------------------------------------------------------------

// The `fusion:` section of the evaluation yaml plus the pipeline-side EvaluationConfig it is
// parsed alongside. Only `arbiter.*` is read from yaml -- multi_camera_fusion and
// crosswalk_estimator are hardcoded to their production defaults below, mirroring
// declare_fusion_config() in traffic_light_fusion_node.cpp: no deployment has ever needed to
// change either from those defaults.
struct FusionEvaluationConfig
{
  std::string output_topic;
  TrafficLightFusionConfig fusion;
};

FusionEvaluationConfig parse_fusion(const YAML::Node & root)
{
  FusionEvaluationConfig config;
  const auto fusion_node = require(root, "fusion");
  config.output_topic = require(fusion_node, "output_topic").as<std::string>();

  // Fixed values, not parameters: mirrors declare_fusion_config()'s hardcoded
  // multi_camera_fusion / crosswalk_estimator defaults (traffic_light_fusion_node.cpp).
  config.fusion.multi_camera_fusion.message_lifespan = 0.09;
  config.fusion.multi_camera_fusion.prior_log_odds = 0.0;
  config.fusion.multi_camera_fusion.use_signal_consistency_check = false;
  config.fusion.multi_camera_fusion.publish_partial_matched_signal = false;

  const auto arbiter_node = require(fusion_node, "arbiter");
  config.fusion.arbiter.external_delay_tolerance =
    require(arbiter_node, "external_delay_tolerance").as<double>();
  config.fusion.arbiter.external_time_tolerance =
    require(arbiter_node, "external_time_tolerance").as<double>();
  config.fusion.arbiter.perception_time_tolerance =
    require(arbiter_node, "perception_time_tolerance").as<double>();
  config.fusion.arbiter.enable_signal_matching =
    require(arbiter_node, "enable_signal_matching").as<bool>();

  auto source_priority = require(arbiter_node, "source_priority").as<std::string>();
  if (
    source_priority != "external" && source_priority != "perception" &&
    source_priority != "confidence") {
    std::cerr << "evaluation config: unknown fusion.arbiter.source_priority '" << source_priority
              << "', defaulting to 'confidence'" << std::endl;
    source_priority = "confidence";
  }
  config.fusion.arbiter.source_priority = source_priority;

  config.fusion.crosswalk_estimator.use_last_detect_color = true;
  config.fusion.crosswalk_estimator.use_pedestrian_signal_detect = true;
  config.fusion.crosswalk_estimator.last_detect_color_hold_time = 2.0;
  config.fusion.crosswalk_estimator.flashing_detection.last_colors_hold_time = 1.0;

  return config;
}

// --- pass A: front-end --------------------------------------------------------------------------

// One front-end run() result, kept together with the camera it came from and the camera_info it
// was produced from -- the back-end pass needs the latter (fusion.run()'s first argument) without
// re-reading the bag.
struct RecordedFrameResult
{
  std::size_t camera_index;
  sensor_msgs::msg::CameraInfo camera_info;
  TrafficLightRecognitionResult result;
};

// Builds one TrafficLightRecognition per camera and drives it over `frames`, which already
// interleaves every camera's frames in ascending stamp order. Frames that fail are logged to
// stderr and skipped, exactly as the Node drops them.
std::vector<RecordedFrameResult> run_recognition(
  const EvaluationConfig & config, const std::vector<Frame> & frames,
  const autoware_map_msgs::msg::LaneletMapBin & map_msg, const tf2::BufferCore & tf_buffer)
{
  std::vector<std::unique_ptr<TrafficLightRecognition>> recognitions;
  for (const auto & camera : config.cameras) {
    auto recognition_config = config.recognition;
    recognition_config.min_timestamp_offset = camera.min_timestamp_offset;
    recognition_config.max_timestamp_offset = camera.max_timestamp_offset;
    recognitions.push_back(
      std::make_unique<TrafficLightRecognition>(recognition_config, map_msg, tf_buffer));
  }

  std::vector<RecordedFrameResult> recorded_results;
  for (const auto & frame : frames) {
    const auto image = decode_frame_image(frame);
    if (!image) {
      continue;
    }
    const auto result = recognitions[frame.camera_index]->run(*image, frame.camera_info);
    if (!result) {
      std::cerr << "camera " << config.cameras[frame.camera_index].ns << " frame at "
                << rclcpp::Time(image->header.stamp).nanoseconds() << " failed: " << result.error()
                << std::endl;
      continue;
    }
    recorded_results.push_back({frame.camera_index, frame.camera_info, *result});
  }
  return recorded_results;
}

// --- pass B: back-end ----------------------------------------------------------------------------

// One TrafficLightFusion, fed pass A's results in the exact order they were produced (which is
// already ascending (stamp, camera_index) order, matching production's per-trigger arrival
// order) -- required because TrafficLightFusion is stateful (see traffic_light_fusion.hpp).
// Events that fail are logged to stderr and skipped.
std::vector<autoware_perception_msgs::msg::TrafficLightGroupArray> run_fusion(
  const FusionEvaluationConfig & fusion_config,
  const std::vector<RecordedFrameResult> & recorded_frame_results,
  const autoware_map_msgs::msg::LaneletMapBin & map_msg)
{
  TrafficLightFusion fusion(fusion_config.fusion, map_msg);

  std::vector<autoware_perception_msgs::msg::TrafficLightGroupArray> recorded_fusion_results;
  for (const auto & recorded : recorded_frame_results) {
    const auto result = fusion.run(
      recorded.camera_info, recorded.result.selected_rois, recorded.result.merged_signals);
    if (!result) {
      std::cerr << "fusion event at "
                << rclcpp::Time(recorded.camera_info.header.stamp).nanoseconds()
                << " failed: " << result.error() << std::endl;
      continue;
    }
    recorded_fusion_results.push_back(*result);
  }
  return recorded_fusion_results;
}

// --- pass C: rosbag output -----------------------------------------------------------------------

// Writes every front-end and back-end result to `output_bag_path` under the configured production
// topic names. The input image/camera_info topics are not copied over, and neither are the cores'
// intermediate stages. Each message is written at its own header stamp (never wall-clock time),
// so the same dataset always produces the same bag.
void write_to_rosbag(
  const EvaluationConfig & config, const FusionEvaluationConfig & fusion_config,
  const std::string & output_bag_path,
  const std::vector<RecordedFrameResult> & recorded_frame_results,
  const std::vector<autoware_perception_msgs::msg::TrafficLightGroupArray> &
    recorded_fusion_results)
{
  autoware::traffic_light::evaluation::remove_output_bag_if_exists(output_bag_path);

  rosbag2_cpp::Writer writer;
  writer.open(
    {output_bag_path,
     autoware::traffic_light::evaluation::detect_input_bag_storage_id(config.input_bag_path)});

  for (const auto & recorded : recorded_frame_results) {
    const auto & camera = config.cameras[recorded.camera_index];
    const rclcpp::Time stamp(recorded.result.merged_signals.header.stamp);
    writer.write(recorded.result.merged_signals, camera.traffic_signals_topic, stamp);
    writer.write(recorded.result.selected_rois, camera.rois_topic, stamp);
  }
  for (const auto & fusion_result : recorded_fusion_results) {
    writer.write(fusion_result, fusion_config.output_topic, rclcpp::Time(fusion_result.stamp));
  }
}

// --- entry point --------------------------------------------------------------------------------

struct CommandLineArgs
{
  std::string config_path;
  std::string dataset_path;
  std::string output_bag_path;
};

CommandLineArgs parse_args(int argc, char ** argv)
{
  CommandLineArgs args;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--config" && i + 1 < argc) {
      args.config_path = argv[++i];
    } else if (arg == "--dataset" && i + 1 < argc) {
      args.dataset_path = argv[++i];
    } else if (arg == "--output-bag" && i + 1 < argc) {
      args.output_bag_path = argv[++i];
    }
  }
  if (args.config_path.empty() || args.dataset_path.empty() || args.output_bag_path.empty()) {
    throw std::runtime_error(
      "usage: run_traffic_light_pipeline_evaluation --config <path> --dataset <path> "
      "--output-bag <path>");
  }
  return args;
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    const auto args = parse_args(argc, argv);
    const auto config = autoware::traffic_light::evaluation::load_evaluation_config(
      args.config_path, args.dataset_path);
    const auto fusion_config = parse_fusion(YAML::LoadFile(args.config_path));

    const auto map_msg = autoware::traffic_light::evaluation::load_map(config);
    // Must outlive every TrafficLightRecognition built from it (its constructor requires this).
    const auto tf_buffer =
      autoware::traffic_light::evaluation::load_transform_buffer(config.input_bag_path);
    const auto frames = autoware::traffic_light::evaluation::load_frames(config);
    std::cerr << "loaded " << frames.size() << " frames from " << config.input_bag_path
              << std::endl;

    const auto recorded_frame_results = run_recognition(config, frames, map_msg, *tf_buffer);
    std::cerr << "front-end: " << recorded_frame_results.size() << " results" << std::endl;

    const auto recorded_fusion_results = run_fusion(fusion_config, recorded_frame_results, map_msg);
    std::cerr << "back-end: " << recorded_fusion_results.size() << " results" << std::endl;

    write_to_rosbag(
      config, fusion_config, args.output_bag_path, recorded_frame_results, recorded_fusion_results);
    std::cerr << "wrote results to " << args.output_bag_path << std::endl;
  } catch (const std::exception & e) {
    std::cerr << "run_traffic_light_pipeline_evaluation failed: " << e.what() << std::endl;
    return 1;
  }
  return 0;
}
