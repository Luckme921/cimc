cmake_minimum_required(VERSION 3.16)
project(chishine_live_viewer VERSION 1.0.0 LANGUAGES C CXX)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux" OR
   NOT CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64|AMD64)$")
  message(FATAL_ERROR
    "This viewer currently targets Ubuntu Linux x86_64 only.")
endif()

# 默认复用同级 x86_chishine_camera_test 中已经验证过的 SDK；也可以在
# cmake 命令中用 -DCHISHINE_SDK_ROOT=/path/to/vendor_sdk 覆盖。
set(CHISHINE_SDK_ROOT
  "${CMAKE_CURRENT_SOURCE_DIR}/../x86_chishine_camera_test/vendor_sdk"
  CACHE PATH "Chishine 3DCamera SDK root")

set(CHISHINE_INCLUDE_DIR "${CHISHINE_SDK_ROOT}/inc/3dcamera")
set(CHISHINE_BUNDLED_LIBRARY
  "${CHISHINE_SDK_ROOT}/lib/3dcamera/linux/x64/lib3DCamera.so")

if(NOT EXISTS "${CHISHINE_INCLUDE_DIR}/3DCamera.hpp")
  message(FATAL_ERROR
    "Missing ${CHISHINE_INCLUDE_DIR}/3DCamera.hpp. Set CHISHINE_SDK_ROOT correctly.")
endif()

# 优先使用已经安装到 /usr/local/lib 并经 ldconfig 注册的正式运行库；
# 若尚未安装，则退回到 vendor_sdk 内的同版本库。
find_library(CHISHINE_3DCAMERA_LIBRARY
  NAMES 3DCamera
  PATHS /usr/local/lib /usr/lib /usr/lib/x86_64-linux-gnu
  NO_DEFAULT_PATH)
if(NOT CHISHINE_3DCAMERA_LIBRARY)
  if(EXISTS "${CHISHINE_BUNDLED_LIBRARY}")
    set(CHISHINE_3DCAMERA_LIBRARY "${CHISHINE_BUNDLED_LIBRARY}")
  else()
    message(FATAL_ERROR "Cannot find lib3DCamera.so")
  endif()
endif()

find_package(PCL 1.10 REQUIRED COMPONENTS common visualization)
add_definitions(${PCL_DEFINITIONS})

add_executable(chishine_live_viewer src/chishine_live_viewer.cpp)
target_compile_features(chishine_live_viewer PRIVATE cxx_std_17)
target_compile_definitions(chishine_live_viewer PRIVATE LINUX_OS)
target_compile_options(chishine_live_viewer PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(chishine_live_viewer PRIVATE
  "${CHISHINE_INCLUDE_DIR}"
  ${PCL_INCLUDE_DIRS})
target_link_directories(chishine_live_viewer PRIVATE ${PCL_LIBRARY_DIRS})
target_link_libraries(chishine_live_viewer PRIVATE
  "${CHISHINE_3DCAMERA_LIBRARY}"
  ${PCL_LIBRARIES})

# 构建目录中直接运行时也能找到未安装的厂商库。正式安装推荐使用
# /usr/local/lib/lib3DCamera.so + sudo ldconfig。
get_filename_component(CHISHINE_LIBRARY_DIR
  "${CHISHINE_3DCAMERA_LIBRARY}" DIRECTORY)
set_target_properties(chishine_live_viewer PROPERTIES
  BUILD_RPATH "${CHISHINE_LIBRARY_DIR}"
  INSTALL_RPATH "/usr/local/lib")

install(TARGETS chishine_live_viewer RUNTIME DESTINATION bin)
install(FILES README.md DESTINATION share/chishine_live_viewer)
