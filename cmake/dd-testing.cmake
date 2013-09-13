if(NOT BUILD_TESTING)
  return()
endif()

include(CTest)

configure_file(${CMAKE_SOURCE_DIR}/CMake/CTestCustom.cmake.in
               ${CMAKE_BINARY_DIR}/CTestCustom.cmake COPYONLY)
