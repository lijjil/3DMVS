#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "OpenMVS::Common" for configuration "Release"
set_property(TARGET OpenMVS::Common APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(OpenMVS::Common PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "/usr/local/openMVS_install/lib/OpenMVS/libCommon.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS OpenMVS::Common )
list(APPEND _IMPORT_CHECK_FILES_FOR_OpenMVS::Common "/usr/local/openMVS_install/lib/OpenMVS/libCommon.a" )

# Import target "OpenMVS::Math" for configuration "Release"
set_property(TARGET OpenMVS::Math APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(OpenMVS::Math PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "/usr/local/openMVS_install/lib/OpenMVS/libMath.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS OpenMVS::Math )
list(APPEND _IMPORT_CHECK_FILES_FOR_OpenMVS::Math "/usr/local/openMVS_install/lib/OpenMVS/libMath.a" )

# Import target "OpenMVS::IO" for configuration "Release"
set_property(TARGET OpenMVS::IO APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(OpenMVS::IO PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "/usr/local/openMVS_install/lib/OpenMVS/libIO.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS OpenMVS::IO )
list(APPEND _IMPORT_CHECK_FILES_FOR_OpenMVS::IO "/usr/local/openMVS_install/lib/OpenMVS/libIO.a" )

# Import target "OpenMVS::MVS" for configuration "Release"
set_property(TARGET OpenMVS::MVS APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(OpenMVS::MVS PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CUDA;CXX"
  IMPORTED_LOCATION_RELEASE "/usr/local/openMVS_install/lib/OpenMVS/libMVS.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS OpenMVS::MVS )
list(APPEND _IMPORT_CHECK_FILES_FOR_OpenMVS::MVS "/usr/local/openMVS_install/lib/OpenMVS/libMVS.a" )

# Import target "OpenMVS::InterfaceCOLMAP" for configuration "Release"
set_property(TARGET OpenMVS::InterfaceCOLMAP APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(OpenMVS::InterfaceCOLMAP PROPERTIES
  IMPORTED_LOCATION_RELEASE "/usr/local/openMVS_install/bin/OpenMVS/InterfaceCOLMAP"
  )

list(APPEND _IMPORT_CHECK_TARGETS OpenMVS::InterfaceCOLMAP )
list(APPEND _IMPORT_CHECK_FILES_FOR_OpenMVS::InterfaceCOLMAP "/usr/local/openMVS_install/bin/OpenMVS/InterfaceCOLMAP" )

# Import target "OpenMVS::InterfaceMetashape" for configuration "Release"
set_property(TARGET OpenMVS::InterfaceMetashape APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(OpenMVS::InterfaceMetashape PROPERTIES
  IMPORTED_LOCATION_RELEASE "/usr/local/openMVS_install/bin/OpenMVS/InterfaceMetashape"
  )

list(APPEND _IMPORT_CHECK_TARGETS OpenMVS::InterfaceMetashape )
list(APPEND _IMPORT_CHECK_FILES_FOR_OpenMVS::InterfaceMetashape "/usr/local/openMVS_install/bin/OpenMVS/InterfaceMetashape" )

# Import target "OpenMVS::InterfaceMVSNet" for configuration "Release"
set_property(TARGET OpenMVS::InterfaceMVSNet APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(OpenMVS::InterfaceMVSNet PROPERTIES
  IMPORTED_LOCATION_RELEASE "/usr/local/openMVS_install/bin/OpenMVS/InterfaceMVSNet"
  )

list(APPEND _IMPORT_CHECK_TARGETS OpenMVS::InterfaceMVSNet )
list(APPEND _IMPORT_CHECK_FILES_FOR_OpenMVS::InterfaceMVSNet "/usr/local/openMVS_install/bin/OpenMVS/InterfaceMVSNet" )

# Import target "OpenMVS::InterfacePolycam" for configuration "Release"
set_property(TARGET OpenMVS::InterfacePolycam APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(OpenMVS::InterfacePolycam PROPERTIES
  IMPORTED_LOCATION_RELEASE "/usr/local/openMVS_install/bin/OpenMVS/InterfacePolycam"
  )

list(APPEND _IMPORT_CHECK_TARGETS OpenMVS::InterfacePolycam )
list(APPEND _IMPORT_CHECK_FILES_FOR_OpenMVS::InterfacePolycam "/usr/local/openMVS_install/bin/OpenMVS/InterfacePolycam" )

# Import target "OpenMVS::DensifyPointCloud" for configuration "Release"
set_property(TARGET OpenMVS::DensifyPointCloud APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(OpenMVS::DensifyPointCloud PROPERTIES
  IMPORTED_LOCATION_RELEASE "/usr/local/openMVS_install/bin/OpenMVS/DensifyPointCloud"
  )

list(APPEND _IMPORT_CHECK_TARGETS OpenMVS::DensifyPointCloud )
list(APPEND _IMPORT_CHECK_FILES_FOR_OpenMVS::DensifyPointCloud "/usr/local/openMVS_install/bin/OpenMVS/DensifyPointCloud" )

# Import target "OpenMVS::ReconstructMesh" for configuration "Release"
set_property(TARGET OpenMVS::ReconstructMesh APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(OpenMVS::ReconstructMesh PROPERTIES
  IMPORTED_LOCATION_RELEASE "/usr/local/openMVS_install/bin/OpenMVS/ReconstructMesh"
  )

list(APPEND _IMPORT_CHECK_TARGETS OpenMVS::ReconstructMesh )
list(APPEND _IMPORT_CHECK_FILES_FOR_OpenMVS::ReconstructMesh "/usr/local/openMVS_install/bin/OpenMVS/ReconstructMesh" )

# Import target "OpenMVS::RefineMesh" for configuration "Release"
set_property(TARGET OpenMVS::RefineMesh APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(OpenMVS::RefineMesh PROPERTIES
  IMPORTED_LOCATION_RELEASE "/usr/local/openMVS_install/bin/OpenMVS/RefineMesh"
  )

list(APPEND _IMPORT_CHECK_TARGETS OpenMVS::RefineMesh )
list(APPEND _IMPORT_CHECK_FILES_FOR_OpenMVS::RefineMesh "/usr/local/openMVS_install/bin/OpenMVS/RefineMesh" )

# Import target "OpenMVS::TextureMesh" for configuration "Release"
set_property(TARGET OpenMVS::TextureMesh APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(OpenMVS::TextureMesh PROPERTIES
  IMPORTED_LOCATION_RELEASE "/usr/local/openMVS_install/bin/OpenMVS/TextureMesh"
  )

list(APPEND _IMPORT_CHECK_TARGETS OpenMVS::TextureMesh )
list(APPEND _IMPORT_CHECK_FILES_FOR_OpenMVS::TextureMesh "/usr/local/openMVS_install/bin/OpenMVS/TextureMesh" )

# Import target "OpenMVS::TransformScene" for configuration "Release"
set_property(TARGET OpenMVS::TransformScene APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(OpenMVS::TransformScene PROPERTIES
  IMPORTED_LOCATION_RELEASE "/usr/local/openMVS_install/bin/OpenMVS/TransformScene"
  )

list(APPEND _IMPORT_CHECK_TARGETS OpenMVS::TransformScene )
list(APPEND _IMPORT_CHECK_FILES_FOR_OpenMVS::TransformScene "/usr/local/openMVS_install/bin/OpenMVS/TransformScene" )

# Import target "OpenMVS::Viewer" for configuration "Release"
set_property(TARGET OpenMVS::Viewer APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(OpenMVS::Viewer PROPERTIES
  IMPORTED_LOCATION_RELEASE "/usr/local/openMVS_install/bin/OpenMVS/Viewer"
  )

list(APPEND _IMPORT_CHECK_TARGETS OpenMVS::Viewer )
list(APPEND _IMPORT_CHECK_FILES_FOR_OpenMVS::Viewer "/usr/local/openMVS_install/bin/OpenMVS/Viewer" )

# Import target "OpenMVS::Tests" for configuration "Release"
set_property(TARGET OpenMVS::Tests APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(OpenMVS::Tests PROPERTIES
  IMPORTED_LOCATION_RELEASE "/usr/local/openMVS_install/bin/OpenMVS/Tests"
  )

list(APPEND _IMPORT_CHECK_TARGETS OpenMVS::Tests )
list(APPEND _IMPORT_CHECK_FILES_FOR_OpenMVS::Tests "/usr/local/openMVS_install/bin/OpenMVS/Tests" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
