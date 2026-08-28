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

#include "traffic_light_recognition/traffic_light_recognition.hpp"

#include <autoware/image_transport_decompressor/image_decompression.hpp>
#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/map_projection_loader/map_projection_loader.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/time.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_filter.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <tf2/buffer_core.h>
#include <tf2/time.h>
#include <yaml-cpp/yaml.h>

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace
{
using autoware::traffic_light::TrafficLightRecognition;
using autoware::traffic_light::TrafficLightRecognitionConfig;
using autoware::traffic_light::TrafficLightRecognitionResult;

// --- evaluation config (yaml) -----------------------------------------------------------------

// One camera's worth of evaluation configuration: which topics it is read from, which topics its
// results are written to, and the one part of TrafficLightRecognitionConfig that legitimately
// differs per camera (the map->camera tf sampling window).
struct CameraConfig
{
  std::string ns;
  std::string camera_info_topic;
  std::optional<std::string> compressed_image_topic;
  std::optional<std::string> image_topic;
  std::string traffic_signals_topic;
  std::string rois_topic;
  double min_timestamp_offset = 0.0;
  double max_timestamp_offset = 0.0;
};

// The whole evaluation run: N cameras sharing one set of models/thresholds, over one dataset.
struct EvaluationConfig
{
  // Derived from --dataset, not from the yaml (fixed dataset layout, see the file header).
  std::string input_bag_path;
  std::string lanelet2_map_path;
  std::string map_projector_info_path;

  std::vector<CameraConfig> cameras;
  // Every field except min/max_timestamp_offset, which CameraConfig carries per camera.
  TrafficLightRecognitionConfig recognition;
};

YAML::Node require(const YAML::Node & node, const std::string & key)
{
  const auto child = node[key];
  if (!child) {
    throw std::runtime_error("evaluation config: missing required key '" + key + "'");
  }
  return child;
}

std::optional<std::string> optional_topic(const YAML::Node & node, const std::string & key)
{
  const auto child = node[key];
  if (!child || child.IsNull()) {
    return std::nullopt;
  }
  return child.as<std::string>();
}

CameraConfig parse_camera(const YAML::Node & node)
{
  CameraConfig camera;
  camera.ns = require(node, "namespace").as<std::string>();
  camera.camera_info_topic = require(node, "camera_info_topic").as<std::string>();
  camera.compressed_image_topic = optional_topic(node, "compressed_image_topic");
  camera.image_topic = optional_topic(node, "image_topic");
  if (camera.compressed_image_topic.has_value() == camera.image_topic.has_value()) {
    throw std::runtime_error(
      "evaluation config: camera '" + camera.ns +
      "' must set exactly one of compressed_image_topic / image_topic");
  }

  const auto output_topics = require(node, "output_topics");
  camera.traffic_signals_topic = require(output_topics, "traffic_signals").as<std::string>();
  camera.rois_topic = require(output_topics, "rois").as<std::string>();

  const auto map_based_detector = require(node, "map_based_detector");
  camera.min_timestamp_offset = require(map_based_detector, "min_timestamp_offset").as<double>();
  camera.max_timestamp_offset = require(map_based_detector, "max_timestamp_offset").as<double>();
  return camera;
}

// Fills everything but min/max_timestamp_offset, which is per camera (see CameraConfig).
TrafficLightRecognitionConfig parse_recognition(const YAML::Node & node)
{
  TrafficLightRecognitionConfig config;

  const auto detector = require(node, "whole_image_detector");
  config.whole_image_detector_model_path = require(detector, "model_path").as<std::string>();
  config.whole_image_detector_label_path = require(detector, "label_path").as<std::string>();
  config.whole_image_detector_roi_remap_path =
    detector["roi_remap_path"] ? detector["roi_remap_path"].as<std::string>() : std::string();
  config.whole_image_detector_score_threshold = require(detector, "score_threshold").as<float>();
  config.whole_image_detector_nms_threshold = require(detector, "nms_threshold").as<float>();

  const auto car = require(node, "car_classifier");
  config.car_classifier_model_path = require(car, "model_path").as<std::string>();
  config.car_classifier_label_path = require(car, "label_path").as<std::string>();

  const auto pedestrian = require(node, "pedestrian_classifier");
  config.pedestrian_classifier_model_path = require(pedestrian, "model_path").as<std::string>();
  config.pedestrian_classifier_label_path = require(pedestrian, "label_path").as<std::string>();

  const auto classifier = require(node, "classifier");
  config.over_exposure_threshold = require(classifier, "over_exposure_threshold").as<double>();
  config.under_exposure_threshold = require(classifier, "under_exposure_threshold").as<double>();

  return config;
}

EvaluationConfig load_evaluation_config(
  const std::string & config_path, const std::string & dataset_path)
{
  EvaluationConfig config;
  config.input_bag_path = dataset_path + "/input_bag";
  config.lanelet2_map_path = dataset_path + "/map/lanelet2_map.osm";
  config.map_projector_info_path = dataset_path + "/map/map_projector_info.yaml";

  const auto root = YAML::LoadFile(config_path);
  for (const auto & camera : require(root, "cameras")) {
    config.cameras.push_back(parse_camera(camera));
  }
  if (config.cameras.empty()) {
    throw std::runtime_error("evaluation config: 'cameras' is empty");
  }
  config.recognition = parse_recognition(require(root, "recognition"));
  return config;
}

// --- rosbag input -----------------------------------------------------------------------------

// One exact-stamp matched (image, camera_info) pair, tagged with the camera it came from --
// the same input unit the Node's message_filters::ExactTime sync hands to run().
struct Frame
{
  std::size_t camera_index;
  sensor_msgs::msg::Image image;
  sensor_msgs::msg::CameraInfo camera_info;
};

template <typename MessageT>
MessageT deserialize(const rosbag2_storage::SerializedBagMessageSharedPtr & bag_message)
{
  rclcpp::SerializedMessage serialized_message(*bag_message->serialized_data);
  rclcpp::Serialization<MessageT> serialization;
  MessageT message;
  serialization.deserialize_message(&serialized_message, &message);
  return message;
}

int64_t stamp_nanoseconds(const std_msgs::msg::Header & header)
{
  return rclcpp::Time(header.stamp).nanoseconds();
}

// Which camera a bag topic belongs to and what it carries, resolved once up front so the single
// pass over the bag does not need to re-derive it.
struct TopicRole
{
  std::size_t camera_index;
  bool is_image;
  bool image_is_compressed;
};

std::unordered_map<std::string, TopicRole> build_topic_roles(
  const std::vector<CameraConfig> & cameras)
{
  std::unordered_map<std::string, TopicRole> roles;
  for (std::size_t index = 0; index < cameras.size(); ++index) {
    const auto & camera = cameras[index];
    const bool compressed = camera.compressed_image_topic.has_value();
    const auto & image_topic = compressed ? *camera.compressed_image_topic : *camera.image_topic;
    roles[image_topic] = TopicRole{index, true, compressed};
    roles[camera.camera_info_topic] = TopicRole{index, false, false};
  }
  return roles;
}

// One camera's images/camera_infos keyed by header stamp while the bag is being read, so pairing
// does not depend on how the two topics happened to interleave on disk.
struct CameraBuffers
{
  std::map<int64_t, sensor_msgs::msg::Image> images_by_stamp;
  std::map<int64_t, sensor_msgs::msg::CameraInfo> camera_infos_by_stamp;
};

// Reads every configured camera's image/camera_info topics and returns the exact-stamp matched
// frames, in ascending (stamp, camera_index) order. Messages with no same-stamp partner on the
// other topic are dropped -- the same policy message_filters::ExactTime enforces in production.
std::vector<Frame> load_frames(const EvaluationConfig & config)
{
  const auto topic_roles = build_topic_roles(config.cameras);

  rosbag2_cpp::Reader reader;
  reader.open(config.input_bag_path);
  rosbag2_storage::StorageFilter filter;
  for (const auto & [topic, role] : topic_roles) {
    filter.topics.push_back(topic);
  }
  reader.set_filter(filter);

  std::vector<CameraBuffers> buffers(config.cameras.size());
  while (reader.has_next()) {
    const auto bag_message = reader.read_next();
    const auto role_it = topic_roles.find(bag_message->topic_name);
    if (role_it == topic_roles.end()) {
      continue;
    }
    const TopicRole & role = role_it->second;
    auto & camera_buffers = buffers[role.camera_index];

    if (!role.is_image) {
      auto camera_info = deserialize<sensor_msgs::msg::CameraInfo>(bag_message);
      camera_buffers.camera_infos_by_stamp.emplace(
        stamp_nanoseconds(camera_info.header), std::move(camera_info));
    } else if (role.image_is_compressed) {
      const auto compressed = deserialize<sensor_msgs::msg::CompressedImage>(bag_message);
      auto decoded =
        autoware::image_transport_decompressor::decompress_image(compressed, "default");
      if (!decoded) {
        std::cerr << "failed to decompress image at " << stamp_nanoseconds(compressed.header)
                  << ": " << decoded.error() << std::endl;
        continue;
      }
      camera_buffers.images_by_stamp.emplace(
        stamp_nanoseconds(decoded->header), std::move(*decoded));
    } else {
      auto image = deserialize<sensor_msgs::msg::Image>(bag_message);
      camera_buffers.images_by_stamp.emplace(stamp_nanoseconds(image.header), std::move(image));
    }
  }

  // std::map keeps each camera's messages stamp-sorted, so collecting matched stamps camera by
  // camera into one std::map<(stamp, camera_index)> yields the merged order directly.
  std::map<std::pair<int64_t, std::size_t>, Frame> frames_by_key;
  for (std::size_t index = 0; index < buffers.size(); ++index) {
    for (auto & [stamp, image] : buffers[index].images_by_stamp) {
      auto camera_info_it = buffers[index].camera_infos_by_stamp.find(stamp);
      if (camera_info_it == buffers[index].camera_infos_by_stamp.end()) {
        continue;
      }
      frames_by_key.emplace(
        std::pair{stamp, index}, Frame{index, std::move(image), std::move(camera_info_it->second)});
    }
  }

  std::vector<Frame> frames;
  frames.reserve(frames_by_key.size());
  for (auto & [key, frame] : frames_by_key) {
    frames.push_back(std::move(frame));
  }
  return frames;
}

// The Node gets its map->camera transforms from a tf2_ros::TransformListener; offline they all
// come from the dataset bag instead. The cache time is deliberately far longer than
// tf2::BufferCore's 10 s default: the whole bag's transforms must stay resolvable for the whole
// run, since frames are processed after the bag has been fully read.
std::unique_ptr<tf2::BufferCore> load_transform_buffer(const std::string & bag_path)
{
  auto buffer = std::make_unique<tf2::BufferCore>(tf2::durationFromSec(24 * 60 * 60));

  rosbag2_cpp::Reader reader;
  reader.open(bag_path);
  reader.set_filter(rosbag2_storage::StorageFilter{{"/tf", "/tf_static"}});
  while (reader.has_next()) {
    const auto bag_message = reader.read_next();
    const bool is_static = bag_message->topic_name == "/tf_static";
    const auto message = deserialize<tf2_msgs::msg::TFMessage>(bag_message);
    for (const auto & transform : message.transforms) {
      buffer->setTransform(transform, "run_traffic_light_recognition_evaluation", is_static);
    }
  }
  return buffer;
}

autoware_map_msgs::msg::LaneletMapBin load_map(const EvaluationConfig & config)
{
  const auto projector_info =
    autoware::map_projection_loader::load_info_from_yaml(config.map_projector_info_path);
  if (projector_info.projector_type != autoware_map_msgs::msg::MapProjectorInfo::MGRS) {
    throw std::runtime_error("dataset map_projector_info.yaml is not an MGRS projector");
  }
  const auto map =
    autoware::experimental::lanelet2_utils::load_mgrs_coordinate_map(config.lanelet2_map_path);
  return autoware::experimental::lanelet2_utils::to_autoware_map_msgs(map);
}

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
                << stamp_nanoseconds(frame.image.header) << " failed: " << result.error()
                << std::endl;
      continue;
    }
    recorded_results.push_back({frame.camera_index, *result});
  }
  return recorded_results;
}

// --- rosbag output ------------------------------------------------------------------------------

// The output bag is always written in the same storage format as the input bag: open the input
// bag with rosbag2_cpp's auto-detecting Reader and read back whichever storage plugin id it
// detected from the bag's own metadata.yaml.
std::string detect_input_bag_storage_id(const std::string & bag_path)
{
  rosbag2_cpp::Reader reader;
  reader.open(bag_path);
  return reader.get_metadata().storage_identifier;
}

// Writes every recorded result to `output_bag_path` under the configured production topic names.
// The input image/camera_info topics are not copied over, and neither are the core's intermediate
// stages -- only run()'s own outputs. Each message is written at its own header stamp (never
// wall-clock time), so the same dataset always produces the same bag.
void write_to_rosbag(
  const EvaluationConfig & config, const std::string & output_bag_path,
  const std::vector<RecordedResult> & recorded_results)
{
  rosbag2_cpp::Writer writer;
  writer.open({output_bag_path, detect_input_bag_storage_id(config.input_bag_path)});

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
    const auto config = load_evaluation_config(args.config_path, args.dataset_path);

    const auto map_msg = load_map(config);
    // Must outlive every TrafficLightRecognition built from it (its constructor requires this).
    const auto tf_buffer = load_transform_buffer(config.input_bag_path);
    const auto frames = load_frames(config);
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
