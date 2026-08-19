cmake_minimum_required(VERSION 3.16)
project(chishine_camera_test VERSION 1.0.0 LANGUAGES CXX)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux" OR
   NOT CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64|AMD64)$")
  message(FATAL_ERROR
    "This package contains only the vendor Linux x86_64 library.")
endif()

set(CHISHINE_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/vendor_sdk" CACHE PATH
  "Chishine 3DCamera SDK root")
set(_sdk_include "${CHISHINE_SDK_ROOT}/inc/3dcamera")
set(_sdk_library_dir "${CHISHINE_SDK_ROOT}/lib/3dcamera/linux/x64")
set(_sdk_library "${_sdk_library_dir}/lib3DCamera.so")

if(NOT EXISTS "${_sdk_include}/3DCamera.hpp")
  message(FATAL_ERROR "Missing ${_sdk_include}/3DCamera.hpp")
endif()
if(NOT EXISTS "${_sdk_library}")
  message(FATAL_ERROR "Missing ${_sdk_library}")
endif()

add_library(chishine_3dcamera_vendor SHARED IMPORTED GLOBAL)
set_target_properties(chishine_3dcamera_vendor PROPERTIES
  IMPORTED_LOCATION "${_sdk_library}")

add_executable(chishine_camera_test src/camera_test.cpp)
target_compile_features(chishine_camera_test PRIVATE cxx_std_17)
target_include_directories(chishine_camera_test PRIVATE "${_sdk_include}")
target_link_libraries(chishine_camera_test PRIVATE chishine_3dcamera_vendor)
target_compile_options(chishine_camera_test PRIVATE -Wall -Wextra -Wpedantic)
set_target_properties(chishine_camera_test PROPERTIES
  BUILD_RPATH "$ORIGIN;${_sdk_library_dir}"
  INSTALL_RPATH "$ORIGIN")
add_custom_command(TARGET chishine_camera_test POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
    "${_sdk_library}" "$<TARGET_FILE_DIR:chishine_camera_test>")

install(TARGETS chishine_camera_test RUNTIME DESTINATION bin)
install(FILES "${_sdk_library}" DESTINATION bin)
install(FILES README.md THIRD_PARTY_NOTICES.md DESTINATION share/chishine_camera_test)
