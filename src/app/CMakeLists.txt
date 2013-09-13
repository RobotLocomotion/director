setup_qt4()

find_package(PythonLibs REQUIRED)
find_package(QtPropertyBrowser REQUIRED)
find_package(PythonQt REQUIRED)
find_package(CTKPythonConsole REQUIRED)

include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${CMAKE_SOURCE_DIR}/src/common)
include_directories(${CTK_PYTHONCONSOLE_INCLUDE_DIRS})

qt4_wrap_cpp(moc_srcs
  ddPythonManager.h
  ddMainWindow.h
  ddViewBase.h
  ddViewManager.h
  )

qt4_wrap_ui(ui_srcs
  ddMainWindow.ui
  )

set(srcs
  ${moc_srcs}
  ${ui_srcs}

  ddPythonManager.cpp
  ddMainWindow.cpp
  ddViewBase.cpp
  ddViewManager.cpp
  )

set(deps
  ${QT_LIBRARIES}
  ${CTK_PYTHONCONSOLE_LIBRARIES}
  ${PYTHONQT_LIBRARIES}
  )

add_library(ddApp ${srcs})
target_link_libraries(ddApp ${deps})

add_executable(drake-designer main.cpp)
target_link_libraries(drake-designer ddApp)
