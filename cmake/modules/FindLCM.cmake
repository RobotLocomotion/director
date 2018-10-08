###############################################################################
# Find package: lcm
#
# This sets the following variables:

# <package>_FOUND
# <package>_INCLUDE_DIRS
# <package>_LIBRARIES

find_package(lcm QUIET CONFIG)
if(lcm_FOUND)
  set(LCM_LIBRARIES ${LCM_NAMESPACE}lcm)
  if(TARGET ${LCM_NAMESPACE}lcm-static)
    set(LCM_STATIC_LIBRARIES ${LCM_NAMESPACE}lcm-static)
  endif()
  return()
endif()

macro(pkg_config_find_module varname pkgname header library pathsuffix)

    find_package(PkgConfig)
    pkg_check_modules(${varname}_pkgconfig ${pkgname})

    find_path(${varname}_INCLUDE_DIR ${header}
        HINTS ${${varname}_pkgconfig_INCLUDEDIR}
        PATH_SUFFIXES ${pathsuffix}
        DOC "Path to ${pkgname} include directory")

    find_library(${varname}_LIBRARY ${library} HINTS ${${varname}_pkgconfig_LIBDIR} DOC "Path to ${pkgname} library")
    find_library(${varname}_STATIC_LIBRARY ${CMAKE_STATIC_LIBRARY_PREFIX}${library}${CMAKE_STATIC_LIBRARY_SUFFIX} HINTS ${${varname}_pkgconfig_LIBDIR} DOC "Path to ${pkgname} static library")

    set(${varname}_INCLUDE_DIRS ${${varname}_INCLUDE_DIR})
    set(${varname}_LIBRARIES ${${varname}_LIBRARY})
    set(${varname}_STATIC_LIBRARIES ${${varname}_STATIC_LIBRARY})

    include(FindPackageHandleStandardArgs)
    find_package_handle_standard_args(${varname} DEFAULT_MSG ${varname}_INCLUDE_DIR ${varname}_LIBRARY)

    mark_as_advanced(${varname}_INCLUDE_DIR)
    mark_as_advanced(${varname}_LIBRARY)
    mark_as_advanced(${varname}_STATIC_LIBRARY)

endmacro()


pkg_config_find_module(LCM lcm lcm/lcm.h lcm lcm)

if(LCM_STATIC_LIBRARIES)
  find_package(GLib2)
  list(APPEND LCM_STATIC_LIBRARIES ${GLib2_LIBRARIES})
endif()
