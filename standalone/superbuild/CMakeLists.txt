cmake_minimum_required(VERSION 2.8.5)
project(Superbuild)

# set a default build type if none was specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  # set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

option(USE_EXTERNAL_INSTALL "If enabled, install dependencies to CMAKE_INSTALL_PREFIX." OFF)
mark_as_advanced(USE_EXTERNAL_INSTALL)

include(ExternalProject)
set_property(DIRECTORY PROPERTY EP_PREFIX ${Superbuild_BINARY_DIR})

set(source_prefix ${Superbuild_BINARY_DIR}/src)
set(install_prefix ${Superbuild_BINARY_DIR}/install)
if (USE_EXTERNAL_INSTALL)
  set(install_prefix ${CMAKE_INSTALL_PREFIX})
endif()
include(cmake/externals.cmake)
