cmake_minimum_required(VERSION 3.14)
project(autoware_traffic_light_multi_camera_fusion)

find_package(autoware_cmake REQUIRED)
autoware_package()

include_directories(
  SYSTEM
)

# Core: no ROS Node, no agnocast (component test plan §9). Depends only on message types,
# rclcpp::Time, and lanelet -- so a Component Test harness can link this library alone.
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/multi_camera_fusion.cpp
  src/traffic_light_multi_camera_fusion_process.cpp
  src/signal_validator.cpp
)

# Node: agnocast wiring only, delegates all logic to the core library above.
ament_auto_add_library(${PROJECT_NAME}_node SHARED
  src/traffic_light_multi_camera_fusion_node.cpp
)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
)

autoware_agnocast_wrapper_register_node(${PROJECT_NAME}_node
  PLUGIN "autoware::traffic_light::MultiCameraFusionNode"
  EXECUTABLE traffic_light_multi_camera_fusion_node
  AGNOCAST_EXECUTOR AgnocastOnlyCallbackIsolatedExecutor
  ROS2_EXECUTOR SingleThreadedExecutor
)

if(BUILD_TESTING AND (NOT "$ENV{ENABLE_AGNOCAST}" STREQUAL "1"))
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_ros REQUIRED)

  # Core tests: no ROS Node, no DDS.
  ament_auto_add_gtest(${PROJECT_NAME}_test
    test/test_utils.cpp
    test/test_signal_validator.cpp
    test/test_multi_camera_fusion.cpp
  )
  target_link_libraries(${PROJECT_NAME}_test
    ${PROJECT_NAME}
  )

  ament_add_ros_isolated_gtest(${PROJECT_NAME}_node_test
    test/test_traffic_light_multi_camera_fusion_node.cpp
  )
  target_link_libraries(${PROJECT_NAME}_node_test
    ${PROJECT_NAME}
    ${PROJECT_NAME}_node
  )
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
