###############################################################################
# Find package: lcm
#
# This sets the following variables:

# <package>_FOUND
# <package>_INCLUDE_DIRS
# <package>_LIBRARIES


macro(pkg_config_find_module varname pkgname header library pathsuffix)

    find_package(PkgConfig)
    pkg_check_modules(${varname}_pkgconfig ${pkgname})

    find_path(${varname}_INCLUDE_DIR ${header}
        HINTS ${${varname}_pkgconfig_INCLUDEDIR}
        PATH_SUFFIXES ${pathsuffix}
        DOC "Path to ${pkgname} include directory")

    find_library(${varname}_LIBRARY ${library} HINTS ${${varname}_pkgconfig_LIBDIR} DOC "Path to ${pkgname} library")

    set(${varname}_INCLUDE_DIRS ${${varname}_INCLUDE_DIR})
    set(${varname}_LIBRARIES ${${varname}_LIBRARY})

    include(FindPackageHandleStandardArgs)
    find_package_handle_standard_args(${varname} DEFAULT_MSG ${varname}_INCLUDE_DIR ${varname}_LIBRARY)

    mark_as_advanced(${varname}_INCLUDE_DIR)
    mark_as_advanced(${varname}_LIBRARY)

endmacro()


pkg_config_find_module(LCM lcm lcm/lcm.h lcm lcm)
