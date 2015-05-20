if(DRAKE_DIR)
  set(_source_dir_hint ${DRAKE_DIR})
  set(_lib_dir_hint ${DRAKE_DIR}/pod-build/lib)
endif()

find_path(DRAKE_SOURCE_DIR addpath_drake.m HINTS ${_source_dir_hint} DOC "Drake source directory")

set(DRAKE_LIBRARIES)
set(_library_var_names)

macro(find_drake_library varName name doc)
  find_library(${varName} ${name} HINTS ${_lib_dir_hint} DOC ${doc})
  list(APPEND DRAKE_LIBRARIES ${${varName}})
  list(APPEND _library_var_names ${varName})
endmacro()

find_drake_library(DRAKE_RBM_LIBRARY drakeRBM "Drake RBM library")
find_drake_library(DRAKE_JOINTS_LIBRARY drakeJoints "Drake Joints library")
find_drake_library(DRAKE_SHAPES_LIBRARY drakeShapes "Drake Shapes library")
find_drake_library(DRAKE_CHULL_LIBRARY drakeConvexHull "Drake Convex Hull library")



set(DRAKE_INCLUDE_DIRS
  ${DRAKE_SOURCE_DIR}/util
  ${DRAKE_SOURCE_DIR}/systems/plants
  ${DRAKE_SOURCE_DIR}/systems/plants/joints
  ${DRAKE_SOURCE_DIR}/systems/plants/tinyxml
  )


include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Drake DEFAULT_MSG DRAKE_SOURCE_DIR ${_library_var_names})
mark_as_advanced(DRAKE_SOURCE_DIR ${_library_var_names})
