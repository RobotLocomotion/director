# Find QtPropertyBrowser
#
# You can pass QTPROPERTYBROWSER_DIR to set the root directory that
# contains lib/ and include/QtPropertyBrowser where QtPropertyBrowser is installed.
#
# Sets QTPROPERTYBROWSER_FOUND, QTPROPERTYBROWSER_INCLUDE_DIRS, QTPROPERTYBROWSER_LIBRARIES
#
# The cache variables are QTPROPERTYBROWSER_INCLUDE_DIR AND QTPROPERTYBROWSER_LIBRARY.
#

if(QTPROPERTYBROWSER_DIR)
  set(_include_dir_hint "${QTPROPERTYBROWSER_DIR}/include/QtPropertyBrowser")
  set(_lib_dir_hint "${QTPROPERTYBROWSER_DIR}/lib")
endif()

find_path(QTPROPERTYBROWSER_INCLUDE_DIR qtpropertybrowser.h HINTS ${_include_dir_hint} PATH_SUFFIXES QtPropertyBrowser DOC "Path to the QtPropertyBrowser include directory")
find_library(QTPROPERTYBROWSER_LIBRARY QtPropertyBrowser HINTS ${_lib_dir_hint} DOC "The QtPropertyBrowser library")

set(QTPROPERTYBROWSER_INCLUDE_DIRS ${QTPROPERTYBROWSER_INCLUDE_DIR})
set(QTPROPERTYBROWSER_LIBRARIES ${QTPROPERTYBROWSER_LIBRARY})

include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(QtPropertyBrowser DEFAULT_MSG QTPROPERTYBROWSER_INCLUDE_DIR QTPROPERTYBROWSER_LIBRARY)

mark_as_advanced(QTPROPERTYBROWSER_INCLUDE_DIR)
mark_as_advanced(QTPROPERTYBROWSER_LIBRARY)

