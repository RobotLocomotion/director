find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})


add_library(botpy SHARED botpy.c)
target_link_libraries(botpy ${PYTHON_LIBRARIES})
use_pkg(botpy botpy_pkgconfig_deps bot2-frames)


if(WIN32 AND NOT CYGWIN)
  set_target_properties(botpy PROPERTIES SUFFIX ".pyd")
else()
  set_target_properties(botpy PROPERTIES PREFIX "" SUFFIX ".so")
endif()

set_target_properties(botpy PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${DD_INSTALL_PYTHON_DIR}/ddapp")
install(TARGETS botpy DESTINATION ${DD_INSTALL_PYTHON_DIR}/ddapp)
