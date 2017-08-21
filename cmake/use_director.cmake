#message("${DIRECTOR_SOURCE_DIR}")
#message("${DIRECTOR_LIBRARIES}")
#message("${DIRECTOR_INCLUDE_DIRS}")
#message("${DIRECTOR_PYTHON_EXECUTABLE}")


set(VTK_DIR "${DIRECTOR_VTK_DIR}")
find_package(VTK REQUIRED COMPONENTS
  vtkWrappingPythonCore
  vtkGUISupportQt
  vtkInteractionWidgets
  vtkRenderingAnnotation
  vtkRenderingFreeType
  vtkInteractionStyle
  vtkIOXML
  vtkIOImage
  vtkIOGeometry
  )
include(${VTK_USE_FILE})

include("${DIRECTOR_SOURCE_DIR}/cmake/dd-macros.cmake")
use_cpp11()
setup_qt()
#setup_python()


list(APPEND CMAKE_MODULE_PATH "${DIRECTOR_SOURCE_DIR}/cmake/modules")
find_package(PythonQt REQUIRED)
find_package(CTKPythonConsole REQUIRED)

include_directories(${PYTHONQT_INCLUDE_DIRS})
include_directories(${CTK_PYTHONCONSOLE_INCLUDE_DIRS})
include_directories(${PYTHON_INCLUDE_DIRS})
include_directories(${DIRECTOR_INCLUDE_DIRS})

if(DD_QT_VERSION EQUAL 4)
  include(${QT_USE_FILE})
else()
  include_directories(${Qt5Core_INCLUDE_DIRS})
endif()
