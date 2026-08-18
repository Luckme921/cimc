cmake_minimum_required(VERSION 3.16)
project(chishine_camera_ros2 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(OpenMP QUIET)

if(NOT DEFINED CHISHINE_3D_CAMERA_SDK_ROOT)
  if(DEFINED ENV{CHISHINE_3D_CAMERA_SDK_ROOT})
    set(CHISHINE_3D_CAMERA_SDK_ROOT "$ENV{CHISHINE_3D_CAMERA_SDK_ROOT}")
  else()
    get_filename_component(_sibling_sdk
      "${CMAKE_CURRENT_SOURCE_DIR}/../../../x86_chishine_camera_test/vendor_sdk"
      ABSOLUTE)
    if(EXISTS "${_sibling_sdk}/inc/3dcamera/3DCamera.hpp")
      set(CHISHINE_3D_CAMERA_SDK_ROOT "${_sibling_sdk}")
    else()
      message(FATAL_ERROR
        "Set CHISHINE_3D_CAMERA_SDK_ROOT to the directory containing inc/ and lib/.")
    endif()
  endif()
endif()

set(_sdk_include "${CHISHINE_3D_CAMERA_SDK_ROOT}/inc/3dcamera")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64|ARM64)$")
  set(_sdk_library_dir "${CHISHINE_3D_CAMERA_SDK_ROOT}/lib/3dcamera/linux/aarch64")
else()
  set(_sdk_library_dir "${CHISHINE_3D_CAMERA_SDK_ROOT}/lib/3dcamera/linux/x64")
endif()
set(_sdk_library "${_sdk_library_dir}/lib3DCamera.so")

if(NOT EXISTS "${_sdk_include}/3DCamera.hpp")
  message(FATAL_ERROR "3DCamera.hpp was not found under ${_sdk_include}")
endif()
if(NOT EXISTS "${_sdk_library}")
  message(FATAL_ERROR "3DCamera library was not found: ${_sdk_library}")
endif()

add_library(chishine_3dcamera_vendor SHARED IMPORTED GLOBAL)
set_target_properties(chishine_3dcamera_vendor PROPERTIES
  IMPORTED_LOCATION "${_sdk_library}")

add_executable(chishine_camera_node src/chishine_camera_node.cpp)
target_compile_features(chishine_camera_node PRIVATE cxx_std_17)
target_include_directories(chishine_camera_node PRIVATE "${_sdk_include}")
target_link_libraries(chishine_camera_node chishine_3dcamera_vendor)
if(OpenMP_CXX_FOUND)
  target_link_libraries(chishine_camera_node OpenMP::OpenMP_CXX)
endif()
ament_target_dependencies(chishine_camera_node rclcpp std_msgs std_srvs)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(chishine_camera_node PRIVATE -Wall -Wextra -Wpedantic)
  set_target_properties(chishine_camera_node PROPERTIES
    BUILD_RPATH "${_sdk_library_dir}" INSTALL_RPATH "$ORIGIN")
endif()

install(TARGETS chishine_camera_node DESTINATION lib/${PROJECT_NAME})
install(FILES "${_sdk_library}" DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES README.md THIRD_PARTY_NOTICES.md DESTINATION share/${PROJECT_NAME})
ament_package()
