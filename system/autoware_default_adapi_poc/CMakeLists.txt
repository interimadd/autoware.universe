cmake_minimum_required(VERSION 3.14)
project(autoware_default_adapi_poc)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/adapi.cpp
)

rclcpp_components_register_nodes(${PROJECT_NAME}
  "autoware::default_adapi::AdapiNode"
)

ament_auto_add_executable(adapi_node
  src/adapi_node.cpp
)

ament_auto_package(INSTALL_TO_SHARE launch)
