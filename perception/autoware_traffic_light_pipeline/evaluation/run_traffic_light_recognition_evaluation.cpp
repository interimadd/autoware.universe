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

// Offline evaluation runner for TrafficLightRecognition (this package's ROS-free core).
//
// Given a t4dataset directory it reads every (image, camera_info) pair out of the dataset's
// rosbag, drives TrafficLightRecognition::run() over them frame by frame, and writes each run()
// result to an output rosbag under production topic names -- no rclcpp::init, no executor, no
// DDS anywhere in this file.
//
// This is a simplified port of autoware_traffic_light_component_test's
// run_traffic_light_pipeline executable, restricted to the front-end (this package's core): the
// back-end fusion stage and the ParameterLoader-based per-component parameter merging are not
// reproduced here, because TrafficLightRecognitionConfig is already flat and small enough to be
// written out directly in one evaluation config yaml.
//
// The front-end+back-end counterpart is run_traffic_light_pipeline_evaluation, which shares this
// file's yaml/rosbag-loading plumbing via evaluation_common.{hpp,cpp}.
//
// Usage:
//   run_traffic_light_recognition_evaluation
//     --config <evaluation config yaml>
//     --dataset <t4dataset dir>
//     --output-bag <output bag dir>
//
// The dataset layout is fixed (same convention as the Component Test harness):
//   <dataset>/input_bag
//   <dataset>/map/lanelet2_map.osm
//   <dataset>/map/map_projector_info.yaml

#include "evaluation_common.hpp"
#include "traffic_light_recognition/traffic_light_recognition.hpp"

#include <rclcpp/time.hpp>
#include <rosbag2_cpp/writer.hpp>

#include <cstddef>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
using autoware::traffic_light::TrafficLightRecognition;
using autoware::traffic_light::TrafficLightRecognitionResult;
using autoware::traffic_light::evaluation::EvaluationConfig;
using autoware::traffic_light::evaluation::Frame;

// --- run --------------------------------------------------------------------------------------

// One run() result, kept together with the camera it came from so write_to_rosbag() can look up
// that camera's output topics. Both messages carry the frame's own stamp (run() copies it from
// the input image's header), so no separate stamp needs to be stored here.
struct RecordedResult
{
  std::size_t camera_index;
  TrafficLightRecognitionResult result;
};

// Builds one TrafficLightRecognition per camera and drives it over `frames`, which already
// interleaves every camera's frames in ascending stamp order. Frames that fail are logged to
// stderr and skipped, exactly as the Node drops them.
std::vector<RecordedResult> run_recognition(
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

  std::vector<RecordedResult> recorded_results;
  for (const auto & frame : frames) {
    const auto result = recognitions[frame.camera_index]->run(frame.image, frame.camera_info);
    if (!result) {
      std::cerr << "camera " << config.cameras[frame.camera_index].ns << " frame at "
                << rclcpp::Time(frame.image.header.stamp).nanoseconds()
                << " failed: " << result.error() << std::endl;
      continue;
    }
    recorded_results.push_back({frame.camera_index, *result});
  }
  return recorded_results;
}

// --- rosbag output ------------------------------------------------------------------------------

// Writes every recorded result to `output_bag_path` under the configured production topic names.
// The input image/camera_info topics are not copied over, and neither are the core's intermediate
// stages -- only run()'s own outputs. Each message is written at its own header stamp (never
// wall-clock time), so the same dataset always produces the same bag.
void write_to_rosbag(
  const EvaluationConfig & config, const std::string & output_bag_path,
  const std::vector<RecordedResult> & recorded_results)
{
  autoware::traffic_light::evaluation::remove_output_bag_if_exists(output_bag_path);

  rosbag2_cpp::Writer writer;
  writer.open(
    {output_bag_path,
     autoware::traffic_light::evaluation::detect_input_bag_storage_id(config.input_bag_path)});

  for (const auto & recorded : recorded_results) {
    const auto & camera = config.cameras[recorded.camera_index];
    const rclcpp::Time stamp(recorded.result.merged_signals.header.stamp);
    writer.write(recorded.result.merged_signals, camera.traffic_signals_topic, stamp);
    writer.write(recorded.result.selected_rois, camera.rois_topic, stamp);
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
      "usage: run_traffic_light_recognition_evaluation --config <path> --dataset <path> "
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

    const auto map_msg = autoware::traffic_light::evaluation::load_map(config);
    // Must outlive every TrafficLightRecognition built from it (its constructor requires this).
    const auto tf_buffer =
      autoware::traffic_light::evaluation::load_transform_buffer(config.input_bag_path);
    const auto frames = autoware::traffic_light::evaluation::load_frames(config);
    std::cerr << "loaded " << frames.size() << " frames from " << config.input_bag_path
              << std::endl;

    const auto recorded_results = run_recognition(config, frames, map_msg, *tf_buffer);
    write_to_rosbag(config, args.output_bag_path, recorded_results);
    std::cerr << "wrote " << recorded_results.size() << " results to " << args.output_bag_path
              << std::endl;
  } catch (const std::exception & e) {
    std::cerr << "run_traffic_light_recognition_evaluation failed: " << e.what() << std::endl;
    return 1;
  }
  return 0;
}
