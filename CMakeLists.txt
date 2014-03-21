cmake_minimum_required(VERSION 2.8.7)

# Drake Designer
project(DD)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/modules")

# set project variables
set(DD_INSTALL_BIN_DIR bin)
set(DD_INSTALL_LIB_DIR lib)
set(DD_INSTALL_INCLUDE_DIR include/ddapp)
set(DD_INSTALL_PYTHON_DIR ${DD_INSTALL_LIB_DIR}/python2.7/dist-packages)

set(EXECUTABLE_OUTPUT_PATH "${CMAKE_BINARY_DIR}/${DD_INSTALL_BIN_DIR}")
set(LIBRARY_OUTPUT_PATH "${CMAKE_BINARY_DIR}/${DD_INSTALL_LIB_DIR}")

set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_INSTALL_NAME_DIR ${CMAKE_INSTALL_RPATH})

# declare options
option(BUILD_SHARED_LIBS "Build drake-designer with shared libraries." ON)
option(BUILD_TESTING "Build drake-designer with tests enabled." OFF)

# include cmake scripts
include(cmake/dd-macros.cmake)
include(cmake/dd-testing.cmake)
include(cmake/dd-version.cmake)

# build project
add_subdirectory(src)

# documentation
add_subdirectory(docs)
