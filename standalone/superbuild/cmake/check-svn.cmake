# This script executes an "svn info" command in order to test
# read access on a csail svn server.  The purpose is to make
# sure that svn authentication is working, since svn authentication
# is required to checkout many of the svn externals.
#
# Users who fail the authentication test should run svn manually,
# enter their password at the prompt, and allow svn to store the
# credentials.  For shared machines without stored credentials,
# you can set the DRC_SVN_PASSWORD cmake variable in order to use
# the drc read-only user for svn authentication.

if(NOT DEFINED DRC_SVN_PASSWORD)
  set(DRC_SVN_PASSWORD "" CACHE STRING "Optional password for svn checkout as drc user.")
endif()

set(svn_credentials)
if(DRC_SVN_PASSWORD)
  set(svn_credentials --username drc --password ${DRC_SVN_PASSWORD})
endif()


set(svn_test_command svn info --non-interactive ${svn_credentials} https://svn.csail.mit.edu/drc)

execute_process(COMMAND ${svn_test_command}
                RESULT_VARIABLE svn_failed
                OUTPUT_QUIET
                ERROR_VARIABLE svn_error_output)

if(svn_failed)
  string (REPLACE ";" " " svn_test_command "${svn_test_command}")
  message(FATAL_ERROR "\nRunning \"${svn_test_command}\" failed with:\n${svn_error_output}\nIf this is an authentication issue, you have two options:  1) run the svn command manually and remove the --non-interactive flag, type your password, and allow svn to store these credentials for future use, or 2) set the DRC_SVN_PASSWORD cmake variable in order to use the drc read-only user for svn authentication.\n")
endif()
