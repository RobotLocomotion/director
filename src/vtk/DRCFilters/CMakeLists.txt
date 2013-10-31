
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_BINARY_DIR})


set(sources
  vtkMultisenseSource.cxx
  vtkMapServerSource.cxx
  vtkFrameWidget.cxx
  vtkFrameWidgetRepresentation.cxx
  )


set(VTK_LIBRARIES
  vtkFiltering vtkIO vtkWidgets
  )


#set(Boost_USE_STATIC_LIBS   ON)
set(Boost_USE_MULTITHREADED ON)
find_package(Boost REQUIRED COMPONENTS system thread)
include_directories(${Boost_INCLUDE_DIRS})

find_package(PkgConfig)
pkg_check_modules(drc_pkg_deps maps drc-lcmtypes bot2-frames bot2-core lcm eigen3)

link_directories(${drc_pkg_deps_LIBRARY_DIRS})
include_directories(${drc_pkg_deps_INCLUDE_DIRS})
add_definitions(${drc_pkg_deps_CFLAGS_OTHER})

message("${drc_pkg_deps_INCLUDE_DIRS}")

set(library_name vtkDRCFilters)

add_library(${library_name} ${sources})
target_link_libraries(${library_name} ${drc_pkg_deps_LIBRARIES} ${Boost_LIBRARIES} ${VTK_LIBRARIES})

if(VTK_WRAP_PYTHON)
  include_directories(${VTK_PYTHON_INCLUDE_DIR})
  include(${CMAKE_SOURCE_DIR}/cmake/wrap-python.cmake)
  wrap_python(${library_name} "${sources}")
endif()
