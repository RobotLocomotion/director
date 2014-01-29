# Find ctkPythonConsole
#
# You can pass CTK_PYTHONCONSOLE_DIR to set the root directory that
# contains lib/ and include/ctkPythonConsole where ctkPythonConsole is installed.
#
# Sets:
#    CTK_PYTHONCONSOLE_FOUND
#    CTK_PYTHONCONSOLE_INCLUDE_DIRS
#    CTK_PYTHONCONSOLE_LIBRARIES
#
# The cache variables are CTK_PYTHONCONSOLE_INCLUDE_DIR AND CTK_PYTHONCONSOLE_LIBRARY.
#

if(CTK_PYTHONCONSOLE_DIR)
  set(_include_dir_hint "${CTK_PYTHONCONSOLE_DIR}/include/ctkPythonConsole")
  set(_lib_dir_hint "${CTK_PYTHONCONSOLE_DIR}/lib")
endif()

find_path(CTK_PYTHONCONSOLE_INCLUDE_DIR ctkPythonConsole.h HINTS ${_include_dir_hint} PATH_SUFFIXES ctkPythonConsole DOC "Path to the ctkPythonConsole include directory")
find_library(CTK_PYTHONCONSOLE_LIBRARY ctkPythonConsole HINTS ${_lib_dir_hint} DOC "Path to the ctkPythonConsole library")

set(CTK_PYTHONCONSOLE_INCLUDE_DIRS ${CTK_PYTHONCONSOLE_INCLUDE_DIR})
set(CTK_PYTHONCONSOLE_LIBRARIES ${CTK_PYTHONCONSOLE_LIBRARY})

include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(CTKPythonConsole DEFAULT_MSG CTK_PYTHONCONSOLE_INCLUDE_DIR CTK_PYTHONCONSOLE_LIBRARY)

mark_as_advanced(CTK_PYTHONCONSOLE_INCLUDE_DIR)
mark_as_advanced(CTK_PYTHONCONSOLE_LIBRARY)
