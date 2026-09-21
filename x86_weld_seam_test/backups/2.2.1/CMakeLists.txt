cmake_minimum_required(VERSION 3.16)
project(weld_seam_sdk VERSION 2.2.1 LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING
    "Build type (Debug, Release, RelWithDebInfo or MinSizeRel)" FORCE)
endif()

option(WELD_SEAM_BUILD_SHARED "Build libweld_seam_sdk" ON)
option(WELD_SEAM_BUILD_CLI "Build weld_seam_extractor" ON)
option(WELD_SEAM_NATIVE_OPTIMIZATION
  "Use -march=native on this PC (faster, but the binary is not portable)" OFF)

find_package(PCL 1.8 REQUIRED COMPONENTS
  common io filters segmentation search features)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(OpenMP QUIET)

if(WELD_SEAM_BUILD_SHARED)
  set(_library_type SHARED)
else()
  set(_library_type STATIC)
endif()

add_library(weld_seam_sdk ${_library_type} main.cpp)
add_library(weld_seam::sdk ALIAS weld_seam_sdk)
target_compile_features(weld_seam_sdk PUBLIC cxx_std_17)
target_compile_definitions(weld_seam_sdk PRIVATE
  WELD_SEAM_SDK_NO_MAIN WELD_SEAM_SDK_EXPORTS)
if(WELD_SEAM_BUILD_SHARED)
  target_compile_definitions(weld_seam_sdk PUBLIC WELD_SEAM_SDK_SHARED)
endif()
target_include_directories(weld_seam_sdk
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  PRIVATE ${PCL_INCLUDE_DIRS})
target_link_directories(weld_seam_sdk PRIVATE ${PCL_LIBRARY_DIRS})
target_compile_options(weld_seam_sdk PRIVATE ${PCL_DEFINITIONS})
target_link_libraries(weld_seam_sdk
  PUBLIC Eigen3::Eigen
  PRIVATE ${PCL_LIBRARIES} Threads::Threads)
if(OpenMP_CXX_FOUND)
  target_link_libraries(weld_seam_sdk PRIVATE OpenMP::OpenMP_CXX)
endif()

if(WELD_SEAM_BUILD_CLI)
  add_executable(weld_seam_extractor main.cpp)
  target_compile_features(weld_seam_extractor PRIVATE cxx_std_17)
  target_include_directories(weld_seam_extractor PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include" ${PCL_INCLUDE_DIRS})
  target_link_directories(weld_seam_extractor PRIVATE ${PCL_LIBRARY_DIRS})
  target_compile_options(weld_seam_extractor PRIVATE ${PCL_DEFINITIONS})
  target_link_libraries(weld_seam_extractor PRIVATE
    Eigen3::Eigen ${PCL_LIBRARIES} Threads::Threads)
  if(OpenMP_CXX_FOUND)
    target_link_libraries(weld_seam_extractor PRIVATE OpenMP::OpenMP_CXX)
  endif()
endif()

foreach(_target weld_seam_sdk weld_seam_extractor)
  if(TARGET ${_target})
    if(MSVC)
      target_compile_options(${_target} PRIVATE /utf-8 /W4)
    else()
      target_compile_options(${_target} PRIVATE -Wall -Wextra -Wpedantic)
      if(WELD_SEAM_NATIVE_OPTIMIZATION)
        target_compile_options(${_target} PRIVATE -march=native)
      endif()
    endif()
  endif()
endforeach()

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND
   CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9)
  target_link_libraries(weld_seam_sdk PRIVATE stdc++fs)
  if(TARGET weld_seam_extractor)
    target_link_libraries(weld_seam_extractor PRIVATE stdc++fs)
  endif()
endif()

set_target_properties(weld_seam_sdk PROPERTIES
  EXPORT_NAME sdk
  OUTPUT_NAME weld_seam_sdk
  VERSION ${PROJECT_VERSION}
  SOVERSION ${PROJECT_VERSION_MAJOR}
  POSITION_INDEPENDENT_CODE ON)

install(TARGETS weld_seam_sdk
  EXPORT weld_seam_sdkTargets
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
if(TARGET weld_seam_extractor)
  install(TARGETS weld_seam_extractor DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(FILES config/default.conf
  DESTINATION ${CMAKE_INSTALL_DATADIR}/weld_seam_sdk/config)
install(FILES README.md
  DESTINATION ${CMAKE_INSTALL_DATADIR}/weld_seam_sdk)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/weld_seam_sdkConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion)
configure_package_config_file(
  cmake/weld_seam_sdkConfig.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/weld_seam_sdkConfig.cmake"
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/weld_seam_sdk)
install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/weld_seam_sdkConfig.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/weld_seam_sdkConfigVersion.cmake"
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/weld_seam_sdk)
install(EXPORT weld_seam_sdkTargets
  FILE weld_seam_sdkTargets.cmake
  NAMESPACE weld_seam::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/weld_seam_sdk)

set(CPACK_PACKAGE_NAME "weld_seam_sdk")
set(CPACK_PACKAGE_VENDOR "SCUT_CIMC")
set(CPACK_PACKAGE_FILE_NAME
  "weld_seam_sdk-${PROJECT_VERSION}-${CMAKE_SYSTEM_NAME}-${CMAKE_SYSTEM_PROCESSOR}")
set(CPACK_GENERATOR "TGZ")
include(CPack)
