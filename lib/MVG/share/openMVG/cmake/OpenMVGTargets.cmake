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
foreach(_expectedTarget OpenMVG::cereal OpenMVG::lib_CoinUtils OpenMVG::lib_Osi OpenMVG::lib_clp OpenMVG::lib_OsiClpSolver OpenMVG::openMVG_stlplus OpenMVG::openMVG_lemon OpenMVG::openMVG_easyexif OpenMVG::openMVG_fast OpenMVG::openMVG_camera OpenMVG::openMVG_exif OpenMVG::openMVG_features OpenMVG::openMVG_graph OpenMVG::openMVG_image OpenMVG::openMVG_linearProgramming OpenMVG::openMVG_lInftyComputerVision OpenMVG::openMVG_geodesy OpenMVG::openMVG_geometry OpenMVG::openMVG_matching OpenMVG::openMVG_kvld OpenMVG::openMVG_matching_image_collection OpenMVG::openMVG_multiview OpenMVG::openMVG_numeric OpenMVG::openMVG_robust_estimation OpenMVG::openMVG_system OpenMVG::openMVG_sfm OpenMVG::vlsift)
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


# Compute the installation prefix relative to this file.
get_filename_component(_IMPORT_PREFIX "${CMAKE_CURRENT_LIST_FILE}" PATH)
get_filename_component(_IMPORT_PREFIX "${_IMPORT_PREFIX}" PATH)
get_filename_component(_IMPORT_PREFIX "${_IMPORT_PREFIX}" PATH)
get_filename_component(_IMPORT_PREFIX "${_IMPORT_PREFIX}" PATH)
if(_IMPORT_PREFIX STREQUAL "/")
  set(_IMPORT_PREFIX "")
endif()

# Create imported target OpenMVG::cereal
add_library(OpenMVG::cereal INTERFACE IMPORTED)

set_target_properties(OpenMVG::cereal PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "${_IMPORT_PREFIX}/include/openMVG_dependencies/cereal/include"
)

# Create imported target OpenMVG::lib_CoinUtils
add_library(OpenMVG::lib_CoinUtils STATIC IMPORTED)

# Create imported target OpenMVG::lib_Osi
add_library(OpenMVG::lib_Osi STATIC IMPORTED)

# Create imported target OpenMVG::lib_clp
add_library(OpenMVG::lib_clp STATIC IMPORTED)

# Create imported target OpenMVG::lib_OsiClpSolver
add_library(OpenMVG::lib_OsiClpSolver STATIC IMPORTED)

# Create imported target OpenMVG::openMVG_stlplus
add_library(OpenMVG::openMVG_stlplus STATIC IMPORTED)

set_target_properties(OpenMVG::openMVG_stlplus PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "/usr/local/openMVG_1_6_install/include/openMVG;/usr/local/openMVG_1_6_install/include/openMVG/third_party/stlplus3/filesystemSimplified"
)

# Create imported target OpenMVG::openMVG_lemon
add_library(OpenMVG::openMVG_lemon STATIC IMPORTED)

set_target_properties(OpenMVG::openMVG_lemon PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "${_IMPORT_PREFIX}/include/openMVG/third_party/lemon;${_IMPORT_PREFIX}/include/openMVG/third_party"
)

# Create imported target OpenMVG::openMVG_easyexif
add_library(OpenMVG::openMVG_easyexif STATIC IMPORTED)

# Create imported target OpenMVG::openMVG_fast
add_library(OpenMVG::openMVG_fast STATIC IMPORTED)

# Create imported target OpenMVG::openMVG_camera
add_library(OpenMVG::openMVG_camera INTERFACE IMPORTED)

set_target_properties(OpenMVG::openMVG_camera PROPERTIES
  INTERFACE_COMPILE_FEATURES "cxx_std_11"
  INTERFACE_INCLUDE_DIRECTORIES "${_IMPORT_PREFIX}/include"
  INTERFACE_LINK_LIBRARIES "OpenMVG::openMVG_numeric;OpenMVG::cereal;gomp"
)

# Create imported target OpenMVG::openMVG_exif
add_library(OpenMVG::openMVG_exif STATIC IMPORTED)

set_target_properties(OpenMVG::openMVG_exif PROPERTIES
  INTERFACE_COMPILE_FEATURES "cxx_std_11"
  INTERFACE_LINK_LIBRARIES "\$<LINK_ONLY:OpenMVG::openMVG_easyexif>"
)

# Create imported target OpenMVG::openMVG_features
add_library(OpenMVG::openMVG_features STATIC IMPORTED)

set_target_properties(OpenMVG::openMVG_features PROPERTIES
  INTERFACE_COMPILE_FEATURES "cxx_std_11"
  INTERFACE_COMPILE_OPTIONS "-mno-sse2;-mno-sse3;-mno-ssse3;-mno-sse4.1;-mno-sse4.2;-mno-sse4a;-mno-avx;-mno-fma;-mno-bmi2;-mno-avx2;-mno-xop;-mno-fma4;-mno-avx512f;-mno-avx512vl;-mno-avx512pf;-mno-avx512er;-mno-avx512cd;-mno-avx512dq;-mno-avx512bw;-mno-avx512ifma;-mno-avx512vbmi;-fopenmp;-std=c++11"
  INTERFACE_INCLUDE_DIRECTORIES "${_IMPORT_PREFIX}/include;${_IMPORT_PREFIX}/include/openMVG"
  INTERFACE_LINK_LIBRARIES "\$<LINK_ONLY:OpenMVG::openMVG_fast>;\$<LINK_ONLY:OpenMVG::openMVG_stlplus>;gomp;OpenMVG::cereal"
)

# Create imported target OpenMVG::openMVG_graph
add_library(OpenMVG::openMVG_graph INTERFACE IMPORTED)

set_target_properties(OpenMVG::openMVG_graph PROPERTIES
  INTERFACE_COMPILE_FEATURES "cxx_std_11"
  INTERFACE_LINK_LIBRARIES "OpenMVG::openMVG_lemon"
)

# Create imported target OpenMVG::openMVG_image
add_library(OpenMVG::openMVG_image STATIC IMPORTED)

set_target_properties(OpenMVG::openMVG_image PROPERTIES
  INTERFACE_COMPILE_FEATURES "cxx_std_11"
  INTERFACE_INCLUDE_DIRECTORIES "${_IMPORT_PREFIX}/include"
  INTERFACE_LINK_LIBRARIES "OpenMVG::openMVG_numeric;gomp;/usr/lib/x86_64-linux-gnu/libjpeg.so;/usr/lib/x86_64-linux-gnu/libpng.so;/usr/lib/x86_64-linux-gnu/libz.so;/usr/lib/x86_64-linux-gnu/libtiff.so"
)

# Create imported target OpenMVG::openMVG_linearProgramming
add_library(OpenMVG::openMVG_linearProgramming STATIC IMPORTED)

set_target_properties(OpenMVG::openMVG_linearProgramming PROPERTIES
  INTERFACE_COMPILE_FEATURES "cxx_std_11"
  INTERFACE_INCLUDE_DIRECTORIES "${_IMPORT_PREFIX}/include"
  INTERFACE_LINK_LIBRARIES "OpenMVG::openMVG_numeric;\$<LINK_ONLY:OpenMVG::lib_clp>;\$<LINK_ONLY:OpenMVG::lib_OsiClpSolver>;\$<LINK_ONLY:OpenMVG::lib_CoinUtils>;\$<LINK_ONLY:OpenMVG::lib_Osi>"
)

# Create imported target OpenMVG::openMVG_lInftyComputerVision
add_library(OpenMVG::openMVG_lInftyComputerVision STATIC IMPORTED)

set_target_properties(OpenMVG::openMVG_lInftyComputerVision PROPERTIES
  INTERFACE_COMPILE_FEATURES "cxx_std_11"
  INTERFACE_LINK_LIBRARIES "OpenMVG::openMVG_linearProgramming;OpenMVG::openMVG_multiview"
)

# Create imported target OpenMVG::openMVG_geodesy
add_library(OpenMVG::openMVG_geodesy INTERFACE IMPORTED)

set_target_properties(OpenMVG::openMVG_geodesy PROPERTIES
  INTERFACE_COMPILE_FEATURES "cxx_std_11"
  INTERFACE_INCLUDE_DIRECTORIES "${_IMPORT_PREFIX}/include"
  INTERFACE_LINK_LIBRARIES "OpenMVG::openMVG_numeric"
)

# Create imported target OpenMVG::openMVG_geometry
add_library(OpenMVG::openMVG_geometry STATIC IMPORTED)

set_target_properties(OpenMVG::openMVG_geometry PROPERTIES
  INTERFACE_COMPILE_FEATURES "cxx_std_11"
  INTERFACE_INCLUDE_DIRECTORIES "${_IMPORT_PREFIX}/include"
  INTERFACE_LINK_LIBRARIES "OpenMVG::openMVG_numeric;OpenMVG::cereal;\$<LINK_ONLY:OpenMVG::openMVG_linearProgramming>"
)

# Create imported target OpenMVG::openMVG_matching
add_library(OpenMVG::openMVG_matching STATIC IMPORTED)

set_target_properties(OpenMVG::openMVG_matching PROPERTIES
  INTERFACE_COMPILE_FEATURES "cxx_std_11"
  INTERFACE_INCLUDE_DIRECTORIES "${_IMPORT_PREFIX}/include;${_IMPORT_PREFIX}/include/openMVG"
  INTERFACE_LINK_LIBRARIES "\$<LINK_ONLY:OpenMVG::openMVG_stlplus>;\$<LINK_ONLY:gomp>;OpenMVG::openMVG_features;Threads::Threads;OpenMVG::cereal"
)

# Create imported target OpenMVG::openMVG_kvld
add_library(OpenMVG::openMVG_kvld STATIC IMPORTED)

set_target_properties(OpenMVG::openMVG_kvld PROPERTIES
  INTERFACE_COMPILE_FEATURES "cxx_std_11"
  INTERFACE_LINK_LIBRARIES "OpenMVG::openMVG_features;OpenMVG::openMVG_image"
)

# Create imported target OpenMVG::openMVG_matching_image_collection
add_library(OpenMVG::openMVG_matching_image_collection STATIC IMPORTED)

set_target_properties(OpenMVG::openMVG_matching_image_collection PROPERTIES
  INTERFACE_COMPILE_FEATURES "cxx_std_11"
  INTERFACE_INCLUDE_DIRECTORIES "${_IMPORT_PREFIX}/include;${_IMPORT_PREFIX}/include/openMVG"
  INTERFACE_LINK_LIBRARIES "OpenMVG::openMVG_matching;OpenMVG::openMVG_multiview;gomp"
)

# Create imported target OpenMVG::openMVG_multiview
add_library(OpenMVG::openMVG_multiview STATIC IMPORTED)

set_target_properties(OpenMVG::openMVG_multiview PROPERTIES
  INTERFACE_COMPILE_FEATURES "cxx_std_11"
  INTERFACE_INCLUDE_DIRECTORIES "${_IMPORT_PREFIX}/include"
  INTERFACE_LINK_LIBRARIES "OpenMVG::openMVG_numeric;\$<LINK_ONLY:OpenMVG::openMVG_graph>;\$<LINK_ONLY:Ceres::ceres>"
)

# Create imported target OpenMVG::openMVG_numeric
add_library(OpenMVG::openMVG_numeric STATIC IMPORTED)

set_target_properties(OpenMVG::openMVG_numeric PROPERTIES
  INTERFACE_COMPILE_FEATURES "cxx_std_11"
  INTERFACE_COMPILE_OPTIONS "-mno-sse2;-mno-sse3;-mno-ssse3;-mno-sse4.1;-mno-sse4.2;-mno-sse4a;-mno-avx;-mno-fma;-mno-bmi2;-mno-avx2;-mno-xop;-mno-fma4;-mno-avx512f;-mno-avx512vl;-mno-avx512pf;-mno-avx512er;-mno-avx512cd;-mno-avx512dq;-mno-avx512bw;-mno-avx512ifma;-mno-avx512vbmi;-fopenmp;-std=c++11"
  INTERFACE_INCLUDE_DIRECTORIES "${_IMPORT_PREFIX}/include;/usr/local/include/eigen3"
)

# Create imported target OpenMVG::openMVG_robust_estimation
add_library(OpenMVG::openMVG_robust_estimation STATIC IMPORTED)

set_target_properties(OpenMVG::openMVG_robust_estimation PROPERTIES
  INTERFACE_COMPILE_FEATURES "cxx_std_11"
  INTERFACE_INCLUDE_DIRECTORIES "${_IMPORT_PREFIX}/include;${_IMPORT_PREFIX}/include/openMVG"
  INTERFACE_LINK_LIBRARIES "OpenMVG::openMVG_numeric"
)

# Create imported target OpenMVG::openMVG_system
add_library(OpenMVG::openMVG_system STATIC IMPORTED)

set_target_properties(OpenMVG::openMVG_system PROPERTIES
  INTERFACE_COMPILE_FEATURES "cxx_std_11"
)

# Create imported target OpenMVG::openMVG_sfm
add_library(OpenMVG::openMVG_sfm STATIC IMPORTED)

set_target_properties(OpenMVG::openMVG_sfm PROPERTIES
  INTERFACE_COMPILE_FEATURES "cxx_std_11"
  INTERFACE_INCLUDE_DIRECTORIES "${_IMPORT_PREFIX}/include;${_IMPORT_PREFIX}/include/openMVG"
  INTERFACE_LINK_LIBRARIES "OpenMVG::openMVG_geometry;OpenMVG::openMVG_features;OpenMVG::openMVG_graph;OpenMVG::openMVG_matching;OpenMVG::openMVG_multiview;OpenMVG::cereal;gomp;\$<LINK_ONLY:Ceres::ceres>;\$<LINK_ONLY:OpenMVG::openMVG_graph>;\$<LINK_ONLY:OpenMVG::openMVG_image>;\$<LINK_ONLY:OpenMVG::openMVG_lInftyComputerVision>;\$<LINK_ONLY:OpenMVG::openMVG_system>;\$<LINK_ONLY:OpenMVG::openMVG_stlplus>"
)

# Create imported target OpenMVG::vlsift
add_library(OpenMVG::vlsift STATIC IMPORTED)

if(CMAKE_VERSION VERSION_LESS 3.0.0)
  message(FATAL_ERROR "This file relies on consumers using CMake 3.0.0 or greater.")
endif()

# Load information for each installed configuration.
get_filename_component(_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
file(GLOB CONFIG_FILES "${_DIR}/OpenMVGTargets-*.cmake")
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
