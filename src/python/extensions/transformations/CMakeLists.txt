find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

find_package(NumPy)
if (NOT NUMPY_FOUND)
  message(WARNING "Optimized transformations C extension module is disabled because numpy is not found.")
  return()
endif()

if (NUMPY_VERSION VERSION_LESS 1.7)
  message(WARNING "Optimized transformations C extension module is disabled because numpy version < 1.7 was found.")
  return()
endif()


include_directories(${NUMPY_INCLUDE_DIRS})

add_library(_transformations SHARED transformations.c)
target_link_libraries(_transformations ${PYTHON_LIBRARIES})


if(WIN32 AND NOT CYGWIN)
  set_target_properties(_transformations PROPERTIES SUFFIX ".pyd")
else()
  set_target_properties(_transformations PROPERTIES PREFIX "" SUFFIX ".so")
endif()

set_target_properties(_transformations PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${DD_INSTALL_PYTHON_DIR}/ddapp/thirdparty")
install(TARGETS _transformations DESTINATION ${DD_INSTALL_PYTHON_DIR}/ddapp/thirdparty)
