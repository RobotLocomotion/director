project(doxygen)

find_package(Doxygen)

if(NOT DOXYGEN_FOUND)
  message(STATUS "Doxygen not found - API documentation will not be built.")
  return()
endif()

set(DOXYGEN_VTK_TAG_FILE "${CMAKE_CURRENT_BINARY_DIR}/vtkNightlyDoc.tag")
set(TAGFILES "\"${DOXYGEN_VTK_TAG_FILE}=http://www.vtk.org/doc/nightly/html\"")


configure_file(${doxygen_SOURCE_DIR}/Doxyfile.in ${doxygen_BINARY_DIR}/Doxyfile)

add_custom_target(docs-doxygen
  COMMAND ${CMAKE_COMMAND} "-DVTK_TAG:FILEPATH=\"${DOXYGEN_VTK_TAG_FILE}\"" -P "${doxygen_SOURCE_DIR}/downloadTags.cmake"
  COMMAND ${CMAKE_COMMAND} -E remove_directory ${doxygen_BINARY_DIR}/html
  COMMAND ${DOXYGEN_EXECUTABLE} ${doxygen_BINARY_DIR}/Doxyfile
  )
