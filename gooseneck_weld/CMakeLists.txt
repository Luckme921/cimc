cmake_minimum_required(VERSION 3.16)
project(gooseneck_sdk VERSION 1.0.0 LANGUAGES C CXX)
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
find_package(PCL REQUIRED COMPONENTS common io)
add_library(gooseneck_sdk SHARED sdk.cpp)
target_compile_features(gooseneck_sdk PUBLIC cxx_std_17)
target_include_directories(gooseneck_sdk PUBLIC
 $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
 $<INSTALL_INTERFACE:include>
 $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../x86_weld_seam_test/include>
 PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(gooseneck_sdk PRIVATE ${PCL_LIBRARIES})
set_target_properties(gooseneck_sdk PROPERTIES VERSION ${PROJECT_VERSION} SOVERSION 1 EXPORT_NAME sdk)
add_executable(gooseneck_weld_extractor cli.cpp)
target_link_libraries(gooseneck_weld_extractor PRIVATE gooseneck_sdk)
set_target_properties(gooseneck_weld_extractor PROPERTIES INSTALL_RPATH "$ORIGIN/../lib")
install(TARGETS gooseneck_sdk gooseneck_weld_extractor EXPORT gooseneck_sdkTargets
 LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/../x86_weld_seam_test/include/weld_seam_sdk/weld_seam_sdk.hpp"
 DESTINATION include/weld_seam_sdk)
write_basic_package_version_file("${CMAKE_CURRENT_BINARY_DIR}/gooseneck_sdkConfigVersion.cmake"
 VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion)
configure_package_config_file(cmake/gooseneck_sdkConfig.cmake.in
 "${CMAKE_CURRENT_BINARY_DIR}/gooseneck_sdkConfig.cmake" INSTALL_DESTINATION lib/cmake/gooseneck_sdk)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/gooseneck_sdkConfig.cmake"
 "${CMAKE_CURRENT_BINARY_DIR}/gooseneck_sdkConfigVersion.cmake" DESTINATION lib/cmake/gooseneck_sdk)
install(EXPORT gooseneck_sdkTargets NAMESPACE gooseneck:: DESTINATION lib/cmake/gooseneck_sdk)
