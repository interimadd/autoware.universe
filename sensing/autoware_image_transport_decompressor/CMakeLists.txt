cmake_minimum_required(VERSION 3.14)
project(autoware_image_transport_decompressor)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(OpenCV REQUIRED)
include_directories(
  ${OpenCV_INCLUDE_DIRS}
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/image_decompression.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
)

ament_auto_add_library(${PROJECT_NAME}_node SHARED
  src/image_transport_decompressor_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "autoware::image_transport_decompressor::ImageTransportDecompressor"
  EXECUTABLE image_transport_decompressor_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_auto_add_gtest(test_image_decompression
    test/test_image_decompression.cpp
  )
  target_link_libraries(test_image_decompression
    ${PROJECT_NAME}
    ${OpenCV_LIBRARIES}
  )
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
