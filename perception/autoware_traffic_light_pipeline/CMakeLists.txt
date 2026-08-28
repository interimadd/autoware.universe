cmake_minimum_required(VERSION 3.14)
project(autoware_traffic_light_pipeline)

find_package(autoware_cmake REQUIRED)
autoware_package()

# This package hosts both halves of the traffic light pipeline, each in its own src/ subdirectory
# and its own pair of libraries (ROS-free core + Node adapter):
#   src/traffic_light_recognition/ -- the front-end (前段): map_based_detector ->
#     whole_image_detector(yolox) -> selector -> car/pedestrian classifier -> category_merger
#   src/traffic_light_fusion/      -- the back-end (後段): multi_camera_fusion -> arbiter ->
#     crosswalk_traffic_light_estimator
# The library names are the component names rather than ${PROJECT_NAME}, since ${PROJECT_NAME}
# now covers two independent compositions.

# --- front-end (traffic_light_recognition) ---------------------------------------------------

# ROS 非依存の合成コア
ament_auto_add_library(autoware_traffic_light_recognition SHARED
  src/traffic_light_recognition/traffic_light_recognition.cpp
)

# Node アダプタ（param 変換を含む）
ament_auto_add_library(autoware_traffic_light_recognition_node SHARED
  src/traffic_light_recognition/traffic_light_recognition_node.cpp
)
target_link_libraries(autoware_traffic_light_recognition_node autoware_traffic_light_recognition)

rclcpp_components_register_node(autoware_traffic_light_recognition_node
  PLUGIN "autoware::traffic_light::TrafficLightRecognitionNode"
  EXECUTABLE traffic_light_recognition_node)

# --- back-end (traffic_light_fusion) ----------------------------------------------------------

# ROS 非依存の合成コア
ament_auto_add_library(autoware_traffic_light_fusion SHARED
  src/traffic_light_fusion/traffic_light_fusion.cpp
)

# Node アダプタ（param 変換を含む）
ament_auto_add_library(autoware_traffic_light_fusion_node SHARED
  src/traffic_light_fusion/traffic_light_fusion_node.cpp
)
target_link_libraries(autoware_traffic_light_fusion_node autoware_traffic_light_fusion)

rclcpp_components_register_node(autoware_traffic_light_fusion_node
  PLUGIN "autoware::traffic_light::TrafficLightFusionNode"
  EXECUTABLE traffic_light_fusion_node)

# --- offline evaluation tool -------------------------------------------------------------------

# t4dataset を入力に TrafficLightRecognition::run() を回し、結果を rosbag に書き出すオフライン
# 評価ツール（evaluation/README.md）。ROS ランタイム（rclcpp::init / executor / DDS）は使わず、
# コアライブラリと rosbag2 だけに依存する。
find_package(yaml-cpp REQUIRED)
ament_auto_add_executable(run_traffic_light_recognition_evaluation
  evaluation/run_traffic_light_recognition_evaluation.cpp
)
target_include_directories(run_traffic_light_recognition_evaluation PRIVATE src)
target_link_libraries(run_traffic_light_recognition_evaluation
  autoware_traffic_light_recognition yaml-cpp)
install(DIRECTORY evaluation/config DESTINATION share/${PROJECT_NAME}/evaluation)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # GPU-gated tests (the front-end core builds TensorRT engines in its constructor). Follows
  # autoware_traffic_light_classifier's convention: build only when CUDA/TensorRT are available,
  # and additionally self-skip (GTEST_SKIP) at runtime when the model files or a usable GPU are
  # not found, so the same binary behaves correctly whether or not autoware_data is populated.
  # The back-end tests need neither: none of its three cores touch TensorRT.
  option(CUDA_AVAIL "CUDA available" OFF)
  find_package(CUDA)
  if(CUDA_FOUND)
    set(CUDA_AVAIL ON)
  endif()
  option(TRT_AVAIL "TensorRT available" OFF)
  find_library(NVINFER NAMES nvinfer)
  find_library(NVONNXPARSER nvonnxparser)
  if(NVINFER AND NVONNXPARSER)
    set(TRT_AVAIL ON)
  endif()

  # ROS-free unit tests for the composed front-end core. GPU-gated (see comment above): the core's
  # constructor builds the whole-image detector's and both classifiers' TensorRT engines.
  if(TRT_AVAIL AND CUDA_AVAIL)
    ament_auto_add_gtest(test_traffic_light_recognition
      test/traffic_light_recognition/test_traffic_light_recognition.cpp
      TIMEOUT 600
    )
    target_link_libraries(test_traffic_light_recognition autoware_traffic_light_recognition)
  endif()

  # Node pub/sub integration test. GPU-gated for the same reason as test_traffic_light_recognition.
  if(TRT_AVAIL AND CUDA_AVAIL)
    ament_add_ros_isolated_gtest(test_traffic_light_recognition_integration
      test/traffic_light_recognition/test_traffic_light_recognition_integration.cpp
      TIMEOUT 600
    )
    target_include_directories(test_traffic_light_recognition_integration PRIVATE src)
    target_link_libraries(test_traffic_light_recognition_integration
      autoware_traffic_light_recognition_node)
  endif()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
