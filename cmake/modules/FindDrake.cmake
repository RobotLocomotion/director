if(DRAKE_DIR)
  set(_source_dir_hint ${DRAKE_DIR})
  set(_lib_dir_hint ${DRAKE_DIR}/pod-build/lib)
endif()

find_path(DRAKE_SOURCE_DIR addpath_drake.m HINTS ${_source_dir_hint} DOC "Drake source directory")
find_library(DRAKE_RBM_LIBRARY drakeRBM HINTS ${_lib_dir_hint} DOC "Drake RBM library")
find_library(DRAKE_URDF_RBM_LIBRARY drakeRBMurdf HINTS ${_lib_dir_hint} DOC "Drake URDFRigidBodyManipulator library")
find_library(DRAKE_URDF_INTERFACE_LIBRARY drakeURDFinterface HINTS ${_lib_dir_hint} DOC "Drake urdf_interface library")

set(DRAKE_LIBRARIES
    ${DRAKE_RBM_LIBRARY}
    ${DRAKE_URDF_RBM_LIBRARY}
    ${DRAKE_URDF_INTERFACE_LIBRARY}
  )

set(DRAKE_INCLUDE_DIRS
  ${DRAKE_SOURCE_DIR}/util
  ${DRAKE_SOURCE_DIR}/systems/plants
  ${DRAKE_SOURCE_DIR}/systems/plants/joints
  ${DRAKE_SOURCE_DIR}/systems/plants/tinyxml
  )


include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Drake DEFAULT_MSG DRAKE_SOURCE_DIR DRAKE_RBM_LIBRARY DRAKE_URDF_RBM_LIBRARY DRAKE_URDF_INTERFACE_LIBRARY)
mark_as_advanced(DRAKE_SOURCE_DIR DRAKE_RBM_LIBRARY DRAKE_URDF_RBM_LIBRARY DRAKE_URDF_INTERFACE_LIBRARY)
