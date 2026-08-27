# autoware_traffic_light_recognition

Single-node composition of the `whole_image_detection` traffic-light front-end pipeline:

```text
map_based_detector -> whole_image_detector(yolox) -> selector
  -> car_classifier / pedestrian_classifier -> category_merger
```

This package composes, for one camera, the ROS-free core logic already extracted into
`autoware_tensorrt_yolox`, `autoware_traffic_light_map_based_detector`,
`autoware_traffic_light_selector`, `autoware_traffic_light_classifier`, and
`autoware_traffic_light_category_merger` into a single `rclcpp::Node`, in place of the 5-Node
graph production currently launches per camera. The `fine_detection` path
(`traffic_light_fine_detector` + `traffic_light_occlusion_predictor`), `multi_camera_fusion`,
`arbiter`, and `crosswalk_traffic_light_estimator` are out of scope and remain separate Nodes.

It ships two libraries, the same shape as `autoware_traffic_light_category_merger`:

- `autoware_traffic_light_recognition` -- the ROS-free composed core (`TrafficLightRecognition`).
- `autoware_traffic_light_recognition_node` -- the Node adapter
  (`TrafficLightRecognitionNode`), registered as an `rclcpp_components` plugin.

## Node

### Input / Output

| Direction  | Topic                      | Type                                                        |
| ---------- | -------------------------- | ----------------------------------------------------------- |
| Subscribed | `~/input/image`            | `sensor_msgs/msg/Image` (already decoded)                   |
| Subscribed | `~/input/camera_info`      | `sensor_msgs/msg/CameraInfo`                                |
| Subscribed | `~/input/vector_map`       | `autoware_map_msgs/msg/LaneletMapBin` (transient_local)     |
| Subscribed | `~/input/route`            | `autoware_planning_msgs/msg/LaneletRoute` (transient_local) |
| Published  | `~/output/traffic_signals` | `tier4_perception_msgs/msg/TrafficLightArray`               |
| Published  | `~/output/rois`            | `tier4_perception_msgs/msg/TrafficLightRoiArray`            |
| Published  | `~/output/camera_info`     | `sensor_msgs/msg/CameraInfo` (republished unchanged)        |

`image` and `camera_info` are synchronized with `message_filters::sync_policies::ExactTime`
(same camera driver, so stamps are expected to match exactly; `approximate_sync` is intentionally
not configurable). `camera_info` is republished unchanged so a `topic_tools relay` node is not
needed downstream.

### Initialization order

The composed core's constructor requires the vector map (`TrafficLightMapBasedDetector`'s
constructor takes a `LaneletMapBin`), so the Node cannot build it at construction time:

1. Constructor: declare parameters, build the config, create the tf buffer/listener and every
   subscriber/publisher.
2. `~/input/vector_map` (transient_local, expected once): build `TrafficLightRecognition`. This
   is also where the whole-image detector's and both classifiers' TensorRT engines get built.
3. `~/input/route`: call `set_route()`. If the map has not arrived yet, the route is held and
   applied once the core is built.
4. Synchronized `(image, camera_info)`: if the core is not yet built, warn (throttled) and drop
   the frame.

This means TensorRT engine construction does not start until the vector map arrives, unlike the
per-stage Nodes it replaces (which each build their own engine in their own constructor). In
`logging_simulator` usage the vector map arrives immediately, so this is not expected to matter in
practice; `build_only` (below) exists for cases where it does.

### `build_only`

When `build_only:=true`, the whole-image detector's and both classifiers' TensorRT engines are
built directly (without constructing `TrafficLightRecognition`, so the vector map is not needed)
and the Node exits. This lets a CI engine-prebuild step target this Node the same way it targets
the existing per-stage Nodes.

## Parameters

See [config/traffic_light_recognition.param.yaml](config/traffic_light_recognition.param.yaml)
and [schema/traffic_light_recognition.schema.json](schema/traffic_light_recognition.schema.json).

Three things are true of this package's parameter surface, all deliberate:

- **`classifier_type` is fixed to CNN.** Both `car_classifier` and `pedestrian_classifier` always
  use `autoware::traffic_light::CNNClassifier`; there is no `classifier_type` parameter (HSV /
  lamp-recognizer backends are not composed here). A future backend would add the parameter back
  at that point rather than exposing a knob with only one valid value today.
- **Semantic-segmentation parameters are not exposed.** The traffic-light yolox model has no
  segmentation head, so `is_roi_overlap_segmentation` / `is_publish_color_mask` /
  `overlap_roi_score_threshold` / the semseg color map are fixed internally
  (`declare_whole_image_detector_config()`) rather than declared as parameters.
- **`model_path` / `label_path` / `roi_remap_path` are not in `config/traffic_light_recognition.param.yaml`.**
  They are injected as separate top-level parameters (see
  [launch/traffic_light_recognition.launch.xml](launch/traffic_light_recognition.launch.xml)) so
  the versioned config file never hard-codes a `$HOME/autoware_data`-relative path.

`car_classifier` and `pedestrian_classifier` are read through the same
`declare_classifier_config(node, prefix, model_path, label_path)` function, called once per
prefix -- the only difference between the two calls is the parameter prefix and the model/label
files passed in.

### Per-camera deployment

In `autoware_launch`, this package's config is expected to be split into a shared file (this
package's default, camera-independent) plus a small per-camera override file carrying only the
keys that actually differ between cameras (in practice, just
`map_based_detector.min_timestamp_offset`) -- not a full per-camera copy of every key. That split
is `autoware_launch`'s responsibility, not this package's; this package only ships the
camera-independent default.

## Testing

| Test                                                  | GPU          | Content                                                                                                                                               |
| ----------------------------------------------------- | ------------ | ----------------------------------------------------------------------------------------------------------------------------------------------------- |
| `test_autoware_traffic_light_recognition`             | required     | `TrafficLightRecognition::run()` with an empty map / no route, tf-resolution failure, `set_route()` error propagation                                 |
| `test_autoware_traffic_light_recognition_params`      | not required | `declare_*_config()` against the package's default config, including that `car_classifier` / `pedestrian_classifier` read distinct parameter prefixes |
| `test_autoware_traffic_light_recognition_integration` | required     | Node pub/sub: the 3 production output topics fire                                                                                                     |

GPU-gated tests are additionally compiled only when CUDA and TensorRT are detected
(`TRT_AVAIL AND CUDA_AVAIL` in `CMakeLists.txt`, matching `autoware_traffic_light_classifier`'s
convention), and self-skip (`GTEST_SKIP`) at runtime when the ONNX models are not present under
`autoware_data` or no usable GPU is found, so the same binary behaves correctly whether or not the
environment is fully provisioned.
