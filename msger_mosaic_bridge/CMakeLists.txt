cmake_minimum_required(VERSION 3.8)
project(msger_mosaic_bridge)

# Set compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

# Add RapidJSON support for std::string
add_definitions(-DRAPIDJSON_HAS_STDSTRING=1)

# Find package dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gps_msgs REQUIRED)

# Include directories
include_directories(
  include/${PROJECT_NAME}
  ${CMAKE_SOURCE_DIR}/thirdparty/rapidjson/include
)

# Declare a C++ executable
add_executable(mosaic_adapter_node
  src/main.cpp
  src/mosaic_adapter.cpp
  src/mosaic_client.cpp
  src/conn_manager.cpp
)

# Specify target properties for the executable
set_target_properties(mosaic_adapter_node PROPERTIES OUTPUT_NAME "mosaic_adapter_node" PREFIX "")

# Link against required libraries
ament_target_dependencies(mosaic_adapter_node
  rclcpp
  gps_msgs
  geometry_msgs
)

# Install the launch directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install C++ executables
install(TARGETS
  mosaic_adapter_node
  DESTINATION lib/${PROJECT_NAME}
)

# Linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Package configuration
ament_package()
