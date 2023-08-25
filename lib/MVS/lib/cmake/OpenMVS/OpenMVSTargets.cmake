# Generated by CMake

if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" LESS 2.5)
   message(FATAL_ERROR "CMake >= 2.6.0 required")
endif()
cmake_policy(PUSH)
cmake_policy(VERSION 2.6)
#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Protect against multiple inclusion, which would fail when already imported targets are added once more.
set(_targetsDefined)
set(_targetsNotDefined)
set(_expectedTargets)
foreach(_expectedTarget OpenMVS::Common OpenMVS::Math OpenMVS::IO OpenMVS::MVS OpenMVS::InterfaceCOLMAP OpenMVS::InterfaceMetashape OpenMVS::InterfaceMVSNet OpenMVS::InterfacePolycam OpenMVS::DensifyPointCloud OpenMVS::ReconstructMesh OpenMVS::RefineMesh OpenMVS::TextureMesh OpenMVS::TransformScene OpenMVS::Viewer OpenMVS::Tests)
  list(APPEND _expectedTargets ${_expectedTarget})
  if(NOT TARGET ${_expectedTarget})
    list(APPEND _targetsNotDefined ${_expectedTarget})
  endif()
  if(TARGET ${_expectedTarget})
    list(APPEND _targetsDefined ${_expectedTarget})
  endif()
endforeach()
if("${_targetsDefined}" STREQUAL "${_expectedTargets}")
  unset(_targetsDefined)
  unset(_targetsNotDefined)
  unset(_expectedTargets)
  set(CMAKE_IMPORT_FILE_VERSION)
  cmake_policy(POP)
  return()
endif()
if(NOT "${_targetsDefined}" STREQUAL "")
  message(FATAL_ERROR "Some (but not all) targets in this export set were already defined.\nTargets Defined: ${_targetsDefined}\nTargets not yet defined: ${_targetsNotDefined}\n")
endif()
unset(_targetsDefined)
unset(_targetsNotDefined)
unset(_expectedTargets)


# The installation prefix configured by this project.
set(_IMPORT_PREFIX "/usr/local/openMVS_install")

# Create imported target OpenMVS::Common
add_library(OpenMVS::Common STATIC IMPORTED)

set_target_properties(OpenMVS::Common PROPERTIES
  INTERFACE_LINK_LIBRARIES "Boost::iostreams;Boost::program_options;Boost::system;Boost::serialization;opencv_calib3d;opencv_core;opencv_dnn;opencv_features2d;opencv_flann;opencv_highgui;opencv_imgcodecs;opencv_imgproc;opencv_ml;opencv_objdetect;opencv_photo;opencv_shape;opencv_stitching;opencv_superres;opencv_video;opencv_videoio;opencv_videostab;opencv_viz;opencv_aruco;opencv_bgsegm;opencv_bioinspired;opencv_ccalib;opencv_cvv;opencv_datasets;opencv_dnn_objdetect;opencv_dpm;opencv_face;opencv_freetype;opencv_fuzzy;opencv_hdf;opencv_hfs;opencv_img_hash;opencv_line_descriptor;opencv_optflow;opencv_phase_unwrapping;opencv_plot;opencv_reg;opencv_rgbd;opencv_saliency;opencv_sfm;opencv_stereo;opencv_structured_light;opencv_surface_matching;opencv_text;opencv_tracking;opencv_xfeatures2d;opencv_ximgproc;opencv_xobjdetect;opencv_xphoto"
)

# Create imported target OpenMVS::Math
add_library(OpenMVS::Math STATIC IMPORTED)

set_target_properties(OpenMVS::Math PROPERTIES
  INTERFACE_LINK_LIBRARIES "OpenMVS::Common"
)

# Create imported target OpenMVS::IO
add_library(OpenMVS::IO STATIC IMPORTED)

set_target_properties(OpenMVS::IO PROPERTIES
  INTERFACE_LINK_LIBRARIES "OpenMVS::Common;/usr/lib/x86_64-linux-gnu/libpng.so;/usr/lib/x86_64-linux-gnu/libz.so;/usr/lib/x86_64-linux-gnu/libjpeg.so;/usr/lib/x86_64-linux-gnu/libtiff.so"
)

# Create imported target OpenMVS::MVS
add_library(OpenMVS::MVS STATIC IMPORTED)

set_target_properties(OpenMVS::MVS PROPERTIES
  INTERFACE_LINK_LIBRARIES "\$<LINK_ONLY:OpenMVS::Common>;\$<LINK_ONLY:OpenMVS::Math>;\$<LINK_ONLY:OpenMVS::IO>;\$<LINK_ONLY:CGAL>;\$<LINK_ONLY:Ceres::ceres>;/usr/lib/x86_64-linux-gnu/libcuda.so"
)

# Create imported target OpenMVS::InterfaceCOLMAP
add_executable(OpenMVS::InterfaceCOLMAP IMPORTED)

# Create imported target OpenMVS::InterfaceMetashape
add_executable(OpenMVS::InterfaceMetashape IMPORTED)

# Create imported target OpenMVS::InterfaceMVSNet
add_executable(OpenMVS::InterfaceMVSNet IMPORTED)

# Create imported target OpenMVS::InterfacePolycam
add_executable(OpenMVS::InterfacePolycam IMPORTED)

# Create imported target OpenMVS::DensifyPointCloud
add_executable(OpenMVS::DensifyPointCloud IMPORTED)

# Create imported target OpenMVS::ReconstructMesh
add_executable(OpenMVS::ReconstructMesh IMPORTED)

# Create imported target OpenMVS::RefineMesh
add_executable(OpenMVS::RefineMesh IMPORTED)

# Create imported target OpenMVS::TextureMesh
add_executable(OpenMVS::TextureMesh IMPORTED)

# Create imported target OpenMVS::TransformScene
add_executable(OpenMVS::TransformScene IMPORTED)

# Create imported target OpenMVS::Viewer
add_executable(OpenMVS::Viewer IMPORTED)

# Create imported target OpenMVS::Tests
add_executable(OpenMVS::Tests IMPORTED)

if(CMAKE_VERSION VERSION_LESS 2.8.12)
  message(FATAL_ERROR "This file relies on consumers using CMake 2.8.12 or greater.")
endif()

# Load information for each installed configuration.
get_filename_component(_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
file(GLOB CONFIG_FILES "${_DIR}/OpenMVSTargets-*.cmake")
foreach(f ${CONFIG_FILES})
  include(${f})
endforeach()

# Cleanup temporary variables.
set(_IMPORT_PREFIX)

# Loop over all imported files and verify that they actually exist
foreach(target ${_IMPORT_CHECK_TARGETS} )
  foreach(file ${_IMPORT_CHECK_FILES_FOR_${target}} )
    if(NOT EXISTS "${file}" )
      message(FATAL_ERROR "The imported target \"${target}\" references the file
   \"${file}\"
but this file does not exist.  Possible reasons include:
* The file was deleted, renamed, or moved to another location.
* An install or uninstall procedure did not complete successfully.
* The installation package was faulty and contained
   \"${CMAKE_CURRENT_LIST_FILE}\"
but not all the files it references.
")
    endif()
  endforeach()
  unset(_IMPORT_CHECK_FILES_FOR_${target})
endforeach()
unset(_IMPORT_CHECK_TARGETS)

# This file does not depend on other imported targets which have
# been exported from the same project but in a separate export set.

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
cmake_policy(POP)
