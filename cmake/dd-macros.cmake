macro(setup_python)
  find_package(PythonLibs 3.4 REQUIRED)
  find_package(PythonInterp 3.4 REQUIRED)
endmacro()


macro(setup_qt)
  get_ubuntu_version()

  set(_default_qt_version 4)
  if (ubuntu_version EQUAL 18.04)
    set(_default_qt_version 5)
  endif()

  set(DD_QT_VERSION ${_default_qt_version} CACHE STRING "Selected Qt version")
  set_property(CACHE DD_QT_VERSION PROPERTY STRINGS 4 5)

  if(NOT (DD_QT_VERSION VERSION_EQUAL 4 OR DD_QT_VERSION VERSION_EQUAL 5))
    message(FATAL_ERROR "DD_QT_VERSION set to unsupported value: ${DD_QT_VERSION}")
  endif()

  if(DEFINED Qt5_DIR AND DEFINED QT_QMAKE_EXECUTABLE)
    message(FATAL_ERROR
      "This project should not be configured with both Qt5_DIR and QT_QMAKE_EXECUTABLE options.
  To build with Qt4, specify QT_QMAKE_EXECUTABLE. To build with Qt5, specify  Qt5_DIR.")
  endif()

  if(DD_QT_VERSION EQUAL 4)
    find_package(Qt4 REQUIRED QtCore QtGui QtOpenGL QtScript)
    mark_as_advanced(QT_QMAKE_EXECUTABLE)
  elseif(DD_QT_VERSION EQUAL 5)
    if(APPLE)
      set(qt_homebrew_dir /usr/local/opt/qt/lib/cmake/)
    endif()
    find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets OpenGL
      PATHS ${qt_homebrew_dir})
  else()
    message(FATAL_ERROR "DD_QT_VERSION is set to an unexpected value: ${DD_QT_VERSION}")
  endif()


endmacro()

macro(qt_wrap_ui)
  if(DD_QT_VERSION EQUAL 4)
    qt4_wrap_ui(${ARGN})
  else()
    qt5_wrap_ui(${ARGN})
  endif()
endmacro()

macro(qt_wrap_cpp)
  if(DD_QT_VERSION EQUAL 4)
    qt4_wrap_cpp(${ARGN})
  else()
    qt5_wrap_cpp(${ARGN})
  endif()
endmacro()

macro(qt_add_resources)
  if(DD_QT_VERSION EQUAL 4)
    qt4_add_resources(${ARGN})
  else()
    qt5_add_resources(${ARGN})
  endif()
endmacro()

macro(check_vtk_qt_version)
  if(NOT (VTK_QT_VERSION VERSION_EQUAL DD_QT_VERSION))
    message(FATAL_ERROR "VTK is compiled with a different version of Qt than "
                        "the one requested.\n"
                        "Expected Qt version Qt${DD_QT_VERSION} but "
                        "VTK is compiled with Qt${VTK_QT_VERSION}.")
  endif()
endmacro()

macro(use_cpp11)
  if (CMAKE_COMPILER_IS_GNUCC)
    execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion
                    OUTPUT_VARIABLE GCC_VERSION)
    if (NOT (GCC_VERSION VERSION_GREATER 4.3 OR GCC_VERSION VERSION_EQUAL 4.3))
      message(FATAL_ERROR "requires gcc version >= 4.3")  # to support the c++0x flag below
    elseif(GCC_VERSION VERSION_LESS 4.7)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
    else()
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    endif()
  elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    if (APPLE)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
    endif()
  endif()
endmacro()


macro(get_ubuntu_version)
  find_program(LSB_RELEASE lsb_release)
  mark_as_advanced(LSB_RELEASE)
  set(ubuntu_version)
  if(LSB_RELEASE)
    execute_process(COMMAND ${LSB_RELEASE} -is
        OUTPUT_VARIABLE osname
        OUTPUT_STRIP_TRAILING_WHITESPACE)
    if(osname STREQUAL Ubuntu)
      execute_process(COMMAND ${LSB_RELEASE} -rs
          OUTPUT_VARIABLE ubuntu_version
          OUTPUT_STRIP_TRAILING_WHITESPACE)
    endif()
  endif()
endmacro()


macro(use_pkg target)

  find_package(PkgConfig REQUIRED)

  foreach (pkgname ${ARGN})
    set(cachevar ${pkgname}_pkgconfig)
    pkg_check_modules(${cachevar} ${pkgname})

    if (NOT ${cachevar}_FOUND)
      message(SEND_ERROR "required package ${pkgname} not found. PKG_CONFIG_PATH=$ENV{PKG_CONFIG_PATH}")
    endif()

    string(REPLACE ";" " " _cflags_str "${${cachevar}_CFLAGS}")
    string(REPLACE ";" " " _ldflags_str "${${cachevar}_LDFLAGS}")
    set_property(TARGET ${target} APPEND_STRING PROPERTY COMPILE_FLAGS "${_cflags_str} ")
    set_property(TARGET ${target} APPEND_STRING PROPERTY LINK_FLAGS "${_ldflags_str} ")
    link_directories(${${cachevar}_LIBRARY_DIRS})
    target_link_libraries(${target} ${${cachevar}_LIBRARIES})

  endforeach()

endmacro()

macro(setup_pkg_config_path)
  set(ENV{PKG_CONFIG_PATH} "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig:$ENV{PKG_CONFIG_PATH}")
endmacro()

macro(install_headers)
  install(FILES ${ARGN} DESTINATION ${DD_INSTALL_INCLUDE_DIR})
endmacro()
