cmake_minimum_required(VERSION 3.16)
project(weld_controller LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Threads REQUIRED)

# 优先接受 -DCONTROLCAN_ROOT=/opt/controlcan，也兼容用户手动安装到
# /usr/local/include + /usr/local/lib 或 /usr/include + /usr/lib。
find_path(CONTROLCAN_INCLUDE_DIR
  NAMES controlcan.h
  HINTS "${CONTROLCAN_ROOT}" "$ENV{CONTROLCAN_ROOT}"
  PATH_SUFFIXES include
  PATHS /usr/local/include /usr/include)
find_library(CONTROLCAN_LIBRARY
  NAMES controlcan libcontrolcan
  HINTS "${CONTROLCAN_ROOT}" "$ENV{CONTROLCAN_ROOT}"
  PATH_SUFFIXES lib lib64
  PATHS /usr/local/lib /usr/lib /usr/lib/x86_64-linux-gnu)

if(NOT CONTROLCAN_INCLUDE_DIR OR NOT CONTROLCAN_LIBRARY)
  message(FATAL_ERROR
    "controlcan.h/libcontrolcan.so not found. Install both files, or pass "
    "-DCONTROLCAN_ROOT=/absolute/sdk/root. Header=${CONTROLCAN_INCLUDE_DIR}; "
    "library=${CONTROLCAN_LIBRARY}")
endif()

add_executable(weld_controller_node src/weld_controller_node.cpp)
target_compile_features(weld_controller_node PRIVATE cxx_std_17)
target_include_directories(weld_controller_node PRIVATE ${CONTROLCAN_INCLUDE_DIR})
target_link_libraries(weld_controller_node
  ${CONTROLCAN_LIBRARY} Threads::Threads)
ament_target_dependencies(weld_controller_node rclcpp std_msgs)

# 依用保留并编译，但前期分节点调试时不启动。
add_executable(weld_logic_node src/weld_logic_node.cpp)
target_compile_features(weld_logic_node PRIVATE cxx_std_17)
ament_target_dependencies(weld_logic_node rclcpp std_msgs)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(weld_controller_node PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_options(weld_logic_node PRIVATE -Wall -Wextra -Wpedantic)
endif()

install(TARGETS weld_controller_node weld_logic_node
  DESTINATION lib/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_package()
