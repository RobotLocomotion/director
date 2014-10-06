macro(setup_qt4)
  find_package(Qt4 REQUIRED QtCore QtGui QtOpenGL QtScript)
  include(${QT_USE_FILE})
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


macro(use_pkg target cachevar)
  find_package(PkgConfig)
  pkg_check_modules(${cachevar}_pkgconfig ${ARGN})

  string(REPLACE ";" " " _cflags_str "${${cachevar}_pkgconfig_CFLAGS}")
  string(REPLACE ";" " " _ldflags_str "${${cachevar}_pkgconfig_LDFLAGS}")

  set_target_properties(${target} PROPERTIES COMPILE_FLAGS ${_cflags_str})
  set_target_properties(${target} PROPERTIES LINK_FLAGS ${_ldflags_str})
  link_directories(${${cachevar}_pkgconfig_LIBRARY_DIRS})
  target_link_libraries(${target} ${${cachevar}_pkgconfig_LIBRARIES})

endmacro()

macro(setup_pods_pkg_config_path)
  set(ENV{PKG_CONFIG_PATH} "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig:${PKG_CONFIG_PATH}")
endmacro()

setup_pods_pkg_config_path()
