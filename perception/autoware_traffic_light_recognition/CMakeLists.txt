cmake_minimum_required(VERSION 3.14)
project(autoware_traffic_light_recognition)

find_package(autoware_cmake REQUIRED)
autoware_package()

# ROS 非依存の合成コア
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/traffic_light_recognition.cpp
)

# Node アダプタ（param 変換を含む）
ament_auto_add_library(${PROJECT_NAME}_node SHARED
  src/traffic_light_recognition_node.cpp
)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "autoware::traffic_light::TrafficLightRecognitionNode"
  EXECUTABLE traffic_light_recognition_node)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # GPU-gated tests (the core builds TensorRT engines in its constructor). Follows
  # autoware_traffic_light_classifier's convention: build only when CUDA/TensorRT are available,
  # and additionally self-skip (GTEST_SKIP) at runtime when the model files or a usable GPU are
  # not found, so the same binary behaves correctly whether or not autoware_data is populated.
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

  # ROS-free unit tests for the composed core. GPU-gated (see comment above): the core's
  # constructor builds the whole-image detector's and both classifiers' TensorRT engines.
  if(TRT_AVAIL AND CUDA_AVAIL)
    ament_auto_add_gtest(test_${PROJECT_NAME}
      test/test_traffic_light_recognition.cpp
      TIMEOUT 600
    )
    target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME})
  endif()

  # Node-level param -> config conversion test. No GPU needed: declare_config() only declares
  # parameters, it never builds a TensorRT engine.
  ament_add_ros_isolated_gtest(test_${PROJECT_NAME}_params
    test/test_traffic_light_recognition_params.cpp
  )
  target_include_directories(test_${PROJECT_NAME}_params PRIVATE src)
  target_link_libraries(test_${PROJECT_NAME}_params ${PROJECT_NAME}_node)

  # Node pub/sub integration test. GPU-gated for the same reason as test_traffic_light_recognition.
  if(TRT_AVAIL AND CUDA_AVAIL)
    ament_add_ros_isolated_gtest(test_${PROJECT_NAME}_integration
      test/test_traffic_light_recognition_integration.cpp
      TIMEOUT 600
    )
    target_include_directories(test_${PROJECT_NAME}_integration PRIVATE src)
    target_link_libraries(test_${PROJECT_NAME}_integration ${PROJECT_NAME}_node)
  endif()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
