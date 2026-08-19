#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "weld_seam::sdk" for configuration "Release"
set_property(TARGET weld_seam::sdk APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(weld_seam::sdk PROPERTIES
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "pcl_common;pcl_octree;pcl_io;pcl_sample_consensus;pcl_kdtree;pcl_filters;pcl_ml;pcl_segmentation;pcl_search;pcl_features;Boost::system;Boost::filesystem;Boost::date_time;Boost::iostreams;Boost::serialization;VTK::ChartsCore;VTK::CommonColor;VTK::CommonComputationalGeometry;VTK::CommonCore;VTK::CommonDataModel;VTK::CommonExecutionModel;VTK::CommonMath;VTK::CommonMisc;VTK::CommonTransforms;VTK::FiltersCore;VTK::FiltersExtraction;VTK::FiltersGeneral;VTK::FiltersGeometry;VTK::FiltersModeling;VTK::FiltersSources;VTK::ImagingCore;VTK::ImagingSources;VTK::InteractionImage;VTK::InteractionStyle;VTK::InteractionWidgets;VTK::IOCore;VTK::IOGeometry;VTK::IOImage;VTK::IOLegacy;VTK::IOPLY;VTK::RenderingAnnotation;VTK::RenderingCore;VTK::RenderingContext2D;VTK::RenderingLOD;VTK::RenderingFreeType;VTK::ViewsCore;VTK::ViewsContext2D;VTK::RenderingOpenGL2;VTK::GUISupportQt;FLANN::FLANN"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libweld_seam_sdk.so.2.2.0"
  IMPORTED_SONAME_RELEASE "libweld_seam_sdk.so.2"
  )

list(APPEND _IMPORT_CHECK_TARGETS weld_seam::sdk )
list(APPEND _IMPORT_CHECK_FILES_FOR_weld_seam::sdk "${_IMPORT_PREFIX}/lib/libweld_seam_sdk.so.2.2.0" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
