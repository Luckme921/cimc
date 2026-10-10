cmake_minimum_required(VERSION 3.16)
project(weld_seam_perception LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
# The shell may prepend the old external SDK to LD_LIBRARY_PATH. Resolve the
# workspace SDK explicitly, and fail configuration if it is not installed.
get_filename_component(_weld_workspace_sdk_prefix
  "${CMAKE_CURRENT_SOURCE_DIR}/../../x86_weld_seam_test/install" ABSOLUTE)
find_package(weld_seam_sdk 2.4.10 CONFIG REQUIRED
  PATHS "${_weld_workspace_sdk_prefix}" NO_DEFAULT_PATH)

add_executable(weld_seam_node src/weld_seam_node.cpp)
target_compile_features(weld_seam_node PRIVATE cxx_std_17)
target_link_libraries(weld_seam_node weld_seam::sdk)
# DT_RPATH is intentionally used here: on this Ubuntu controller the old SDK
# is first in LD_LIBRARY_PATH, which would override a normal DT_RUNPATH.
set_target_properties(weld_seam_node PROPERTIES
  INSTALL_RPATH "${_weld_workspace_sdk_prefix}/lib")
if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND
   CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_link_options(weld_seam_node PRIVATE -Wl,--disable-new-dtags)
endif()
ament_target_dependencies(weld_seam_node
  rclcpp std_msgs std_srvs geometry_msgs)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(weld_seam_node PRIVATE -Wall -Wextra -Wpedantic)
endif()

install(TARGETS weld_seam_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_package()
