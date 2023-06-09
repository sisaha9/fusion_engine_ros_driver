cmake_minimum_required(VERSION 3.12)

set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)
set(P1_FE_BUILD_EXAMPLES OFF)

project(fusion_engine_ros_driver)

include(FetchContent)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Specify external project as a target.
FetchContent_Declare(
    fusion_engine_client
    GIT_REPOSITORY https://github.com/PointOneNav/fusion-engine-client.git
    GIT_TAG origin/master
)
FetchContent_Populate(fusion_engine_client)
add_subdirectory(${fusion_engine_client_SOURCE_DIR})

include_directories(${fusion_engine_client_SOURCE_DIR}/src)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

set(${PROJECT_NAME}_SRC
  src/fusion_engine_ros_driver_node.cpp
  src/fusion_engine_ros_driver_utils.cpp
)

# Create this project as a library.
ament_auto_add_library(${PROJECT_NAME} SHARED
  ${${PROJECT_NAME}_SRC}
)

# Register this project as a component node.
set(${PROJECT_NAME}_NODE_PLUGIN_CLASS "point_one::fusion_engine::ros_driver::FusionEngineRosDriverNode")
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN ${${PROJECT_NAME}_NODE_PLUGIN_CLASS}
  EXECUTABLE ${PROJECT_NAME}_node_exe)

# Link to fusion_engine_client library.
target_link_libraries(${PROJECT_NAME} fusion_engine_client)
target_link_libraries(${PROJECT_NAME}_node_exe fusion_engine_client)

if(BUILD_TESTING)
  find_package(ament_cmake_clang_format REQUIRED)
  ament_clang_format(src/ include/ --config ${CMAKE_CURRENT_SOURCE_DIR}/.clang-format)
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  param
)
