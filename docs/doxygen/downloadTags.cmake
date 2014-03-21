
if(NOT VTK_TAG)
  message(FATAL_ERROR "VTK_TAG was not specified")
endif()

message(STATUS "Downloading VTK doxygen tag file")
file(DOWNLOAD http://www.vtk.org/files/nightly/vtkNightlyDoc.tag "${VTK_TAG}" SHOW_PROGRESS)
