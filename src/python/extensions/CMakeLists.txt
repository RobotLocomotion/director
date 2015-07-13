add_subdirectory(transformations)

if(USE_PORTMIDI)
  add_subdirectory(pypm)
endif()

if(USE_LIBBOT)
  add_subdirectory(botpy)
endif()
