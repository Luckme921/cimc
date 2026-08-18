cmake_minimum_required(VERSION 3.16)
project(weld_seam_perception LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(weld_seam_sdk 2.2 CONFIG REQUIRED)

add_executable(weld_seam_node src/weld_seam_node.cpp)
target_compile_features(weld_seam_node PRIVATE cxx_std_17)
target_link_libraries(weld_seam_node weld_seam::sdk)
ament_target_dependencies(weld_seam_node
  rclcpp std_msgs std_srvs geometry_msgs)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(weld_seam_node PRIVATE -Wall -Wextra -Wpedantic)
endif()

install(TARGETS weld_seam_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_package()
