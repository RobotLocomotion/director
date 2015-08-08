

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

find_package(OpenGL REQUIRED)

set(sources
  vtkGridSource.cxx
  vtkFrameWidget.cxx
  vtkFrameWidgetRepresentation.cxx

  vtkInteractorStyleTerrain2.cxx

  vtkDepthImageProcessingPass.cxx
  vtkEDLShading.cxx
  )


#####

set(GLSL_RESOURCES_DIR
    edl_resources/Shaders
    )

set(GLSL_SRCS
  edl_compose
  edl_shade
  bilateral_filter
  depth_compose
  )

# -----------------------------------------------------------------------------
# Create custom commands to encode each glsl file into a C string literal
# in a header file
# -----------------------------------------------------------------------------
set(EXTRA_SRCS)

foreach(file ${GLSL_SRCS})
  set(src ${CMAKE_CURRENT_SOURCE_DIR}/${GLSL_RESOURCES_DIR}/${file}.glsl)
  set(res ${CMAKE_CURRENT_BINARY_DIR}/${file}.cxx)
  add_custom_command(
    OUTPUT ${res}
    DEPENDS ${src}
    COMMAND ${VTK_ENCODESTRING_EXE}
    ARGS ${res} ${src} ${file}
    )
  set(EXTRA_SRCS ${EXTRA_SRCS} ${res})
endforeach(file)


#####

set(VTK_LIBRARIES
  vtkFiltering
  vtkIO
  vtkWidgets
  )


set(deps
  ${VTK_LIBRARIES}
  ${OPENGL_LIBRARIES}
  )

set(pkg_deps)

if (USE_DRC)

  # requires libbot, lcm, eigen, boost, drc lcmtypes
  set(Boost_USE_MULTITHREADED ON)
  find_package(Boost REQUIRED COMPONENTS system thread)
  include_directories(${Boost_INCLUDE_DIRS})

  find_package(LibBot REQUIRED)
  include_directories(${LIBBOT_INCLUDE_DIRS})

  find_package(Eigen REQUIRED)
  include_directories(${EIGEN_INCLUDE_DIRS})

  find_package(LCM REQUIRED)
  include_directories(${LCM_INCLUDE_DIRS})

  list(APPEND sources
    vtkMultisenseSource.cxx
  )

  list(APPEND deps
    ${Boost_LIBRARIES}
    ${LIBBOT_LIBRARIES}
    ${LCM_LIBRARIES}
  )

  list(APPEND pkg_deps
    drc-lcmtypes
  )

endif()

if (USE_DRC_MAPS)
  find_package(Maps REQUIRED)
  find_package(PCL REQUIRED)

  list(APPEND sources vtkMapServerSource.cxx)
  include_directories(${PCL_INCLUDE_DIRS})
  include_directories(${MAPS_INCLUDE_DIRS})
  list(APPEND deps ${MAPS_LIBRARIES} ${PCL_LIBRARIES})
  use_cpp11()
endif()

if (USE_DRC_PLANE_SEG)
  find_package(PlaneSeg REQUIRED)
  find_package(PCL REQUIRED)

  list(APPEND sources vtkPlaneSegmentation.cxx)
  include_directories(${PLANE_SEG_INCLUDE_DIRS})
  include_directories(${PCL_INCLUDE_DIRS})
  list(APPEND deps ${PLANE_SEG_LIBRARIES} ${PCL_LIBRARIES})
  use_cpp11()
endif()

if(USE_LCMGL)

  find_package(OpenGL REQUIRED)
  list(APPEND sources vtkLCMGLProp.cxx)
  list(APPEND deps ${LIBBOT_LIBRARIES} ${OPENGL_LIBRARIES})

  list(APPEND pkg_deps
    bot2-lcmgl-client
    bot2-lcmgl-renderer
  )

endif()

set(library_name vtkDRCFilters)
add_library(${library_name} ${sources} ${EXTRA_SRCS})
target_link_libraries(${library_name} ${deps})

if(pkg_deps)
  use_pkg(${library_name} ${pkg_deps})
endif()

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
