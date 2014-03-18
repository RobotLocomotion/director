
set(Boost_USE_MULTITHREADED ON)
find_package(Boost REQUIRED COMPONENTS system thread)
include_directories(${Boost_INCLUDE_DIRS})


include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_BINARY_DIR})


set(sources
  vtkGridSource.cxx
  vtkFrameWidget.cxx
  vtkFrameWidgetRepresentation.cxx
  vtkMultisenseSource.cxx
  )


set(VTK_LIBRARIES
  vtkFiltering
  vtkIO
  vtkWidgets
  )


set(deps
  ${Boost_LIBRARIES}
  ${VTK_LIBRARIES}
  )

set(pkg_deps
  drc-lcmtypes
  bot2-frames
  bot2-core
  lcm
  eigen3
)


option(USE_DRC_MAPS "Build drake-designer with drc maps dependency." ON)
if (USE_DRC_MAPS)
  find_package(Maps REQUIRED)
  find_package(PCL REQUIRED)

  list(APPEND sources vtkMapServerSource.cxx)
  include_directories(${PCL_INCLUDE_DIRS})
  include_directories(${MAPS_INCLUDE_DIRS})
  list(APPEND deps ${MAPS_LIBRARIES} ${PCL_LIBRARIES})
  use_cpp11()
endif()

option(USE_LCM_GL "Build drake-designer with bot lcm gl dependency." ON)
if(USE_LCM_GL)
  list(APPEND sources vtkLCMGLProp.cxx)
  list(APPEND pkg_deps bot2-lcmgl-renderer)
endif()

set(library_name vtkDRCFilters)
add_library(${library_name} ${sources})
target_link_libraries(${library_name} ${deps})
use_pkg(${library_name} ${library_name}_pkgconfig_deps ${pkg_deps})

install(TARGETS ${library_name}
    RUNTIME DESTINATION ${DD_INSTALL_BIN_DIR}
    LIBRARY DESTINATION ${DD_INSTALL_LIB_DIR}
    ARCHIVE DESTINATION ${DD_INSTALL_LIB_DIR})

if(VTK_WRAP_PYTHON)
  include_directories(${VTK_PYTHON_INCLUDE_DIR})
  include(${CMAKE_SOURCE_DIR}/cmake/wrap-python.cmake)
  wrap_python(${library_name} "${sources}")
  set_target_properties(${library_name}Python PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${DD_INSTALL_PYTHON_DIR}/ddapp")
endif()
