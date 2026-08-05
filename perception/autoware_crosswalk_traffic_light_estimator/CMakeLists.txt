cmake_minimum_required(VERSION 3.5)
project(autoware_crosswalk_traffic_light_estimator)

find_package(autoware_cmake REQUIRED)
autoware_package()

###########
## Build ##
###########

include_directories()

# Core: no ROS Node, no agnocast (component test plan §9). Depends only on message types,
# rclcpp::Time, and lanelet -- so a Component Test harness can link this library alone.
ament_auto_add_library(autoware_crosswalk_traffic_light_estimator SHARED
  src/crosswalk_traffic_light_estimator.cpp
  src/flashing_detection.cpp
)

# Node: agnocast wiring only, delegates all logic to the core library above.
ament_auto_add_library(autoware_crosswalk_traffic_light_estimator_node SHARED
  src/node.cpp
)
target_link_libraries(autoware_crosswalk_traffic_light_estimator_node
  autoware_crosswalk_traffic_light_estimator
)

autoware_agnocast_wrapper_register_node(autoware_crosswalk_traffic_light_estimator_node
  PLUGIN "autoware::crosswalk_traffic_light_estimator::CrosswalkTrafficLightEstimatorNode"
  EXECUTABLE crosswalk_traffic_light_estimator_node
  AGNOCAST_EXECUTOR AgnocastOnlyCallbackIsolatedExecutor
  ROS2_EXECUTOR SingleThreadedExecutor
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest)
  ament_lint_auto_find_test_dependencies()

  if(NOT "$ENV{ENABLE_AGNOCAST}" STREQUAL "1")
    ament_add_gtest(test_crosswalk_traffic_light_estimator_node
      test/test_crosswalk_traffic_light_estimator_node.cpp
    )
    target_link_libraries(test_crosswalk_traffic_light_estimator_node
      autoware_crosswalk_traffic_light_estimator
      autoware_crosswalk_traffic_light_estimator_node
    )
  endif()

  ament_add_gtest(test_flashing_detection
    test/test_flashing_detection.cpp
  )
  target_link_libraries(test_flashing_detection
    autoware_crosswalk_traffic_light_estimator
  )

  ament_add_gtest(test_crosswalk_traffic_light_estimator
    test/test_crosswalk_traffic_light_estimator.cpp
  )
  target_link_libraries(test_crosswalk_traffic_light_estimator
    autoware_crosswalk_traffic_light_estimator
  )

  # Cross-checks the component-test framework's ParameterLoader against the Node's declared
  # parameters (component test plan §10).
  find_package(autoware_component_test_framework REQUIRED)
  ament_add_gtest(test_crosswalk_traffic_light_estimator_params_cross_check
    test/test_crosswalk_traffic_light_estimator_params_cross_check.cpp
  )
  target_include_directories(test_crosswalk_traffic_light_estimator_params_cross_check PRIVATE
    ${autoware_component_test_framework_INCLUDE_DIRS}
  )
  target_link_libraries(test_crosswalk_traffic_light_estimator_params_cross_check
    autoware_crosswalk_traffic_light_estimator
    ${autoware_component_test_framework_LIBRARIES}
  )
endif()

#############
## Install ##
#############

autoware_ament_auto_package(
        INSTALL_TO_SHARE
        launch
        config
)
