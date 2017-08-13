#.rst:
#
# FindGLib2
# ---------
#
# Find the glib-2.0 headers and libraries.
#
# Result variables
# ^^^^^^^^^^^^^^^^
#
# This module will set the following variables in your project:
#
# ``GLib2_FOUND``
#   TRUE if the glib-2.0 headers and libraries were found.
# ``GLib2_VERSION``
#   The glib-2.0 release version.
# ``GLib2_INCLUDE_DIRS``
#   The directory containing the glib-2.0 headers.
# ``GLib2_LIBRARIES``
#   The glib-2.0 libraries to be linked.
#
# Cache variables
# ^^^^^^^^^^^^^^^
#
# The following cache variables may be set:
#
# ``GLib2_GLIB_INCLUDE_DIR``
#   The directory containing the ``glib.h`` header.
# ``GLib2_GLIBCONFIG_INCLUDE_DIR``
#   The directory containing the ``glibconfig.h`` header.
# ``GLib2_LIBRARY``
#   The path to the ``glib-2.0`` library.

find_package(PkgConfig REQUIRED)

set(PKG_CONFIG_USE_CMAKE_PREFIX_PATH ON)
pkg_check_modules(PC_GLib2 glib-2.0)

set(GLib2_VERSION ${PC_GLib2_VERSION})

find_path(GLib2_GLIB_INCLUDE_DIR
    NAMES glib.h
    PATHS "${PC_GLib2_INCLUDE_DIRS}"
    PATH_SUFFIXES glib-2.0 glib-2.0/include
)

find_library(GLib2_LIBRARY
    NAMES glib-2.0 glib
    PATHS "${PC_GLib2_LIBRARY_DIRS}"
)

get_filename_component(GLib2_PREFIX "${GLib2_LIBRARY}" DIRECTORY)

find_path(GLib2_GLIBCONFIG_INCLUDE_DIR
    NAMES glibconfig.h
    PATHS "${GLib2_PREFIX}"
    PATH_SUFFIXES glib-2.0 glib-2.0/include
)

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(GLib2
    REQUIRED_VARS
        GLib2_GLIB_INCLUDE_DIR
        GLib2_GLIBCONFIG_INCLUDE_DIR
        GLib2_LIBRARY
    VERSION_VAR GLib2_VERSION
)

if(GLib2_FOUND)
    set(GLib2_INCLUDE_DIRS
        "${GLib2_GLIB_INCLUDE_DIR}"
        "${GLib2_GLIBCONFIG_INCLUDE_DIR}"
    )

    set(GLib2_LIBRARIES "${GLib2_LIBRARY}")

    mark_as_advanced(
        GLib2_GLIB_INCLUDE_DIR
        GLib2_GLIBCONFIG_INCLUDE_DIR
        GLib2_LIBRARY
    )
endif()
