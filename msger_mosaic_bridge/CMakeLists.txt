cmake_minimum_required(VERSION 3.5)
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
find_package(carma_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ament_cmake_gtest REQUIRED) # Add support for Google Test

# Include directories
include_directories(
  include/${PROJECT_NAME}
  ${CMAKE_SOURCE_DIR}/thirdparty/rapidjson/include
  ${ament_index_build_path}/carma_msgs/include  
)

# Create a library from shared source files
add_library(msger_mosaic_bridge_lib
  src/mosaic_adapter.cpp
  src/mosaic_client.cpp
  src/conn_manager.cpp
)

# Link the shared library against required dependencies
ament_target_dependencies(msger_mosaic_bridge_lib
  rclcpp
  gps_msgs
  geometry_msgs
  carma_msgs
  std_srvs
)

# Declare a C++ executable using the shared library
add_executable(mosaic_adapter_node src/main.cpp)
target_link_libraries(mosaic_adapter_node msger_mosaic_bridge_lib)

# Specify target properties for the executable
set_target_properties(mosaic_adapter_node PROPERTIES OUTPUT_NAME "mosaic_adapter_node" PREFIX "")

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

# Linting and Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Add test executables and link against the shared library
  ament_add_gtest(test_mosaic_adapter test/test_mosaic_adapter.cpp)
  target_link_libraries(test_mosaic_adapter msger_mosaic_bridge_lib)
  ament_target_dependencies(test_mosaic_adapter rclcpp gps_msgs geometry_msgs)

endif()

# Package configuration
ament_package()
