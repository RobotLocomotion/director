find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

# find libbot using pkg-config
find_package(PkgConfig)
pkg_check_modules(pkg_deps bot2-frames bot2-core)

link_directories(${pkg_deps_LIBRARY_DIRS})
include_directories(${pkg_deps_INCLUDE_DIRS})
add_definitions(${pkg_deps_CFLAGS_OTHER})


add_library(botpy SHARED botpy.c)
target_link_libraries(botpy ${pkg_deps_LIBRARIES} ${PYTHON_LIBRARIES})

set_target_properties(botpy PROPERTIES PREFIX "" SUFFIX ".so")

if(WIN32 AND NOT CYGWIN)
  set_target_properties(botpy PROPERTIES SUFFIX ".pyd")
endif()

set_target_properties(botpy PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${LIBRARY_OUTPUT_PATH}/site-packages/ddapp)
