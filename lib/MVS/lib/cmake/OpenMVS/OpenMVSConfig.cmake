# Configure file for the OpenMVS package, defining the following variables:
#  OpenMVS_INCLUDE_DIRS - include directories
#  OpenMVS_DEFINITIONS  - definitions to be used
#  OpenMVS_LIBRARIES    - libraries to link against
#  OpenMVS_BINARIES     - binaries


####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was OpenMVSConfig.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../../../usr/local/openMVS_install" ABSOLUTE)

macro(check_required_components _NAME)
  foreach(comp ${${_NAME}_FIND_COMPONENTS})
    if(NOT ${_NAME}_${comp}_FOUND)
      if(${_NAME}_FIND_REQUIRED_${comp})
        set(${_NAME}_FOUND FALSE)
      endif()
    endif()
  endforeach()
endmacro()

####################################################################################

set(OpenMVS_VERSION "2.2.0")

# Compute paths
set(OpenMVS_PREFIX "/usr/local/openMVS_install")
set(OpenMVS_CMAKE_DIR "/usr/local/openMVS_install/lib/cmake")
set(OpenMVS_INCLUDE_DIRS "/usr/local/openMVS_install/include;/usr/include;/usr/local/include/eigen3;/usr/local/include;/usr/local/include/opencv")

set(OpenMVS_DEFINITIONS "")

# These are IMPORTED targets created by OpenMVSTargets.cmake
set(OpenMVS_LIBRARIES MVS)
set(OpenMVS_BINARIES InterfaceCOLMAP DensifyPointCloud ReconstructMesh RefineMesh TextureMesh)

include("${CMAKE_CURRENT_LIST_DIR}/OpenMVSTargets.cmake")
check_required_components("OpenMVS")
