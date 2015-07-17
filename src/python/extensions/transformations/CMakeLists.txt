find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

get_filename_component(python_lib_dir ${PYTHON_LIBRARY} PATH)
set(_include_dir_hint "${python_lib_dir}/python2.7/site-packages/numpy/core/include")
find_path(NUMPY_INCLUDE_DIR numpy/arrayobject.h HINTS ${_include_dir_hint} DOC "Path to the NumPy include directory")
mark_as_advanced(NUMPY_INCLUDE_DIR)

if (NOT NUMPY_INCLUDE_DIR)
  message(WARNING "Optimized transformations C extension module is disabled because NUMPY_INCLUDE_DIR is not set.")
  return()
endif()


include_directories(${NUMPY_INCLUDE_DIR})
add_library(_transformations SHARED transformations.c)
target_link_libraries(_transformations ${PYTHON_LIBRARIES})


if(WIN32 AND NOT CYGWIN)
  set_target_properties(_transformations PROPERTIES SUFFIX ".pyd")
else()
  set_target_properties(_transformations PROPERTIES PREFIX "" SUFFIX ".so")
endif()

set_target_properties(_transformations PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${DD_INSTALL_PYTHON_DIR}/ddapp/thirdparty")
install(TARGETS _transformations DESTINATION ${DD_INSTALL_PYTHON_DIR}/ddapp/thirdparty)

