find_package(PythonLibs REQUIRED)
find_library(PORTMIDI_LIBRARY portmidi "portmidi library")
find_path(PORTMIDI_INCLUDE_DIR portmidi.h "portmidi include dir")
mark_as_advanced(PORTMIDI_LIBRARY PORTMIDI_INCLUDE_DIR)

include_directories(${PYTHON_INCLUDE_DIRS})
include_directories(${PORTMIDI_INCLUDE_DIR})

add_library(pypm SHARED pypm.c)
target_link_libraries(pypm ${PORTMIDI_LIBRARY} ${PYTHON_LIBRARIES})

set_target_properties(pypm PROPERTIES PREFIX "" SUFFIX ".so")

if(WIN32 AND NOT CYGWIN)
  set_target_properties(pypm PROPERTIES SUFFIX ".pyd")
endif()

set_target_properties(pypm PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${DD_INSTALL_PYTHON_DIR}/ddapp")
install(TARGETS pypm DESTINATION ${DD_INSTALL_PYTHON_DIR}/ddapp)
