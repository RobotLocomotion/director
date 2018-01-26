option(USE_PCL "Build with PCL." OFF)
option(USE_LCM "Build with lcm." OFF)
option(USE_LCMGL "Build with lcm-gl." OFF)
option(USE_OCTOMAP "Build with octomap." OFF)
option(USE_APRILTAGS "Build with apriltags lcm driver." OFF)
option(USE_OPENNI2_LCM "Build with openni2 lcm driver." OFF)
option(USE_COLLECTIONS "Build with collections." OFF)
option(USE_LIBBOT "Build with libbot." OFF)
option(USE_DRAKE "Build with drake." OFF)
option(USE_STANDALONE_LCMGL "Build with standalone bot-lcmgl." OFF)
option(USE_PERCEPTION "Build director features that require OpenCV, PCL, cv-utils, and libbot as dependencies." OFF)

option(USE_SYSTEM_EIGEN "Use system version of eigen.  If off, eigen will be built." OFF)
option(USE_SYSTEM_LCM "Use system version of lcm.  If off, lcm will be built." OFF)
option(USE_SYSTEM_LIBBOT "Use system version of libbot.  If off, libbot will be built." OFF)
option(USE_SYSTEM_PCL "Use system version of pcl.  If off, pcl will be built." OFF)
option(USE_SYSTEM_VTK "Use system version of VTK.  If off, VTK will be built." OFF)
if(NOT USE_SYSTEM_VTK AND NOT APPLE)
  option(USE_PRECOMPILED_VTK "Download and use precompiled VTK.  If off, VTK will be compiled from source." ON)
endif()

option(BUILD_SHARED_LIBS "Build director and externals with shared libraries." ON)

if(USE_DRAKE)
  set(DRAKE_SOURCE_DIR CACHE PATH "")
  set(DRAKE_SUPERBUILD_PREFIX_PATH "")
  if(DRAKE_SOURCE_DIR)
    set(DRAKE_SUPERBUILD_PREFIX_PATH "${DRAKE_SOURCE_DIR}/build/install")
    if(NOT EXISTS "${DRAKE_SUPERBUILD_PREFIX_PATH}")
        message(SEND_ERROR "Cannot find build directory in DRAKE_SOURCE_DIR: ${DRAKE_SOURCE_DIR}")
    endif()
  endif()
endif()

set(default_cmake_args
  "-DCMAKE_PREFIX_PATH:PATH=${install_prefix};${DRAKE_SUPERBUILD_PREFIX_PATH};${CMAKE_PREFIX_PATH}"
  "-DCMAKE_INSTALL_PREFIX:PATH=${install_prefix}"
  "-DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}"
  "-DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=ON"
  "-DBUILD_SHARED_LIBS:BOOL=${BUILD_SHARED_LIBS}"
  "-DBUILD_DOCUMENTATION:BOOL=OFF"
  "-DENABLE_TESTING:BOOL=OFF"
  "-DCMAKE_C_COMPILER_LAUNCHER:FILEPATH=${CMAKE_C_COMPILER_LAUNCHER}"
  "-DCMAKE_C_COMPILER:FILEPATH=${CMAKE_C_COMPILER}"
  "-DCMAKE_C_FLAGS:STRING=${CMAKE_C_FLAGS}"
  "-DCMAKE_CXX_COMPILER_LAUNCHER:FILEPATH=${CMAKE_CXX_COMPILER_LAUNCHER}"
  "-DCMAKE_CXX_COMPILER:FILEPATH=${CMAKE_CXX_COMPILER}"
  "-DCMAKE_CXX_COMPILER:FILEPATH=${CMAKE_CXX_COMPILER}"
  "-DCMAKE_CXX_FLAGS:STRING=${CMAKE_CXX_FLAGS}"
  "-DCMAKE_EXE_LINKER_FLAGS:STRING=${CMAKE_EXE_LINKER_FLAGS}"
  "-DCMAKE_SHARED_LINKER_FLAGS:STRING=${CMAKE_SHARED_LINKER_FLAGS}"
  "-DCMAKE_VERBOSE_MAKEFILE:BOOL=${CMAKE_VERBOSE_MAKEFILE}"
  )

if(APPLE)
  list(APPEND default_cmake_args -DCMAKE_MACOSX_RPATH:BOOL=ON)
  if(CMAKE_INSTALL_RPATH)
    list(APPEND default_cmake_args "-DCMAKE_INSTALL_RPATH:STRING=${CMAKE_INSTALL_RPATH}")
  endif()
endif()

# Find required external dependencies
setup_qt()
if (DD_QT_VERSION EQUAL 4)
  set(qt_args
    -DQT_QMAKE_EXECUTABLE:FILEPATH=${QT_QMAKE_EXECUTABLE}
    )
else()
  set(qt_args
    -DQt5_DIR:PATH=${Qt5_DIR}
    -DQt5Core_DIR:PATH=${Qt5Core_DIR}
    -DQt5Gui_DIR:PATH=${Qt5Gui_DIR}
    -DQt5Widgets_DIR:PATH=${Qt5Widgets_DIR}
    )
endif()


if(APPLE)
  find_program(PYTHON_CONFIG_EXECUTABLE python-config)
  if (NOT PYTHON_CONFIG_EXECUTABLE)
    message(SEND_ERROR "python-config executable not found, but python is required.")
  endif()
  # using "python-config --prefix" so that cmake always uses the python that is
  # in the users path, this is a fix for homebrew on Mac:
  # https://github.com/Homebrew/homebrew/issues/25118
  execute_process(COMMAND ${PYTHON_CONFIG_EXECUTABLE} --prefix OUTPUT_VARIABLE python_prefix OUTPUT_STRIP_TRAILING_WHITESPACE)
  set(PYTHON_INCLUDE_DIR ${python_prefix}/include/python2.7)
  set(PYTHON_LIBRARY ${python_prefix}/lib/libpython2.7${CMAKE_SHARED_LIBRARY_SUFFIX})
else()
  find_package(PythonLibs 2.7 REQUIRED)
endif()
find_package(PythonInterp 2.7 REQUIRED)

set(python_args
  -DPYTHON_EXECUTABLE:PATH=${PYTHON_EXECUTABLE}
  -DPYTHON_INCLUDE_DIR:PATH=${PYTHON_INCLUDE_DIR}
  -DPYTHON_INCLUDE_DIR2:PATH=${PYTHON_INCLUDE_DIR} # required for cmake 2.8 on ubuntu 14.04
  -DPYTHON_LIBRARY:PATH=${PYTHON_LIBRARY}
  )


###############################################################################
# eigen

if (NOT USE_SYSTEM_EIGEN)

ExternalProject_Add(
  eigen
  URL https://bitbucket.org/eigen/eigen/get/3.3.3.tar.gz
  URL_MD5 f21cee193e15e55cfd15ebbc16fc00a7
  CMAKE_CACHE_ARGS
    ${default_cmake_args}
    ${qt_args}
)

ExternalProject_Add_Step(eigen make_pkgconfig_dir
  COMMAND ${CMAKE_COMMAND} -E make_directory ${install_prefix}/lib/pkgconfig
  DEPENDERS configure)

set(eigen_args
  -DEIGEN_INCLUDE_DIR:PATH=${install_prefix}/include/eigen3
  -DEIGEN_INCLUDE_DIRS:PATH=${install_prefix}/include/eigen3
  -DEIGEN3_INCLUDE_DIR:PATH=${install_prefix}/include/eigen3
  )

set(eigen_depends eigen)

endif()


###############################################################################
# lcm

if (USE_LCM AND NOT USE_SYSTEM_LCM)


  if(CMAKE_VERSION VERSION_LESS 3.1)
    ExternalProject_Add(
      cmake3
      URL https://cmake.org/files/v3.5/cmake-3.5.2-Linux-x86_64.tar.gz
      URL_MD5 c7a119aad057a3c0508a2c6d281c6291
      CONFIGURE_COMMAND ""
      BUILD_COMMAND ""
      INSTALL_COMMAND ""
    )
    set(cmake3_args CMAKE_COMMAND ${source_prefix}/cmake3/bin/cmake)
    set(cmake3_depends cmake3)
  endif()

  ExternalProject_Add(lcm
    GIT_REPOSITORY https://github.com/lcm-proj/lcm.git
    GIT_TAG c0a0093
    ${cmake3_args}
    CMAKE_CACHE_ARGS
      -DLCM_ENABLE_EXAMPLES:BOOL=OFF
      -DLCM_ENABLE_LUA:BOOL=OFF
      -DLCM_ENABLE_TESTS:BOOL=OFF
      -DLCM_INSTALL_M4MACROS:BOOL=OFF
      -DLCM_NAMESPACE:STRING=lcm::
      ${default_cmake_args}
      ${python_args}
    DEPENDS
      ${cmake3_depends}
    )

  set(lcm_depends lcm ${cmake3_depends})
endif()


###############################################################################
# libbot

if(USE_LIBBOT AND NOT USE_SYSTEM_LIBBOT)

  if(NOT USE_LCM)
    message(SEND_ERROR "Error, USE_LIBBOT is enabled but USE_LCM is OFF.")
  endif()

  ExternalProject_Add(libbot
    GIT_REPOSITORY https://github.com/RobotLocomotion/libbot2.git
    GIT_TAG 4835477
    CMAKE_CACHE_ARGS
      ${default_cmake_args}
      -DWITH_BOT_VIS:BOOL=OFF

    DEPENDS
      ${lcm_depends}
    )

  set(libbot_depends libbot)

endif()


###############################################################################
# lcm message types repos

if (USE_LCM AND NOT USE_SYSTEM_LIBBOT)

  ExternalProject_Add(bot_core_lcmtypes
    GIT_REPOSITORY https://github.com/openhumanoids/bot_core_lcmtypes
    GIT_TAG 9967654
    ${cmake3_args}
    CMAKE_CACHE_ARGS
      ${default_cmake_args}
      ${python_args}

    DEPENDS
      ${lcm_depends}

      # build bot_core_lcmtypes after libbot, even though it is not a dependency.
      # see https://github.com/RobotLocomotion/libbot/issues/20
      ${libbot_depends}
    )

  ExternalProject_Add(robotlocomotion-lcmtypes
    GIT_REPOSITORY https://github.com/robotlocomotion/lcmtypes.git
    GIT_TAG 821ff4b
    ${cmake3_args}
    CMAKE_CACHE_ARGS
      ${default_cmake_args}
      ${python_args}

    DEPENDS
      ${lcm_depends}
      bot_core_lcmtypes
    )

    list(APPEND lcm_depends bot_core_lcmtypes robotlocomotion-lcmtypes)

endif()


###############################################################################
if(USE_STANDALONE_LCMGL)

  if(USE_LIBBOT)
    message(SEND_ERROR "USE_LIBBOT and USE_STANDALONE_LCMGL are incompatible.  Please disable one options.")
  endif()

  ExternalProject_Add(bot-lcmgl-download
    GIT_REPOSITORY https://github.com/RobotLocomotion/libbot.git
    GIT_TAG 4835477
    SOURCE_DIR ${source_prefix}/bot-lcmgl
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND ""
    )

  ExternalProject_Add(bot-lcmgl
    SOURCE_DIR ${source_prefix}/bot-lcmgl/bot2-lcmgl
    DOWNLOAD_COMMAND ""
    UPDATE_COMMAND ""
    DEPENDS bot-lcmgl-download
    CMAKE_CACHE_ARGS
      ${default_cmake_args}
      -DUSE_BOT_VIS:BOOL=OFF
    )

  set(lcmgl_depends bot-lcmgl)

endif()


###############################################################################
# PythonQt

if(DD_QT_VERSION EQUAL 4)
  set(PythonQt_TAG patched-6)
else()
  set(PythonQt_TAG patched-8)
endif()

ExternalProject_Add(PythonQt
  GIT_REPOSITORY https://github.com/commontk/PythonQt.git
  GIT_TAG ${PythonQt_TAG}
  CMAKE_CACHE_ARGS
    ${default_cmake_args}
    ${qt_args}
    ${python_args}
    -DPythonQt_Wrap_Qtcore:BOOL=ON
    -DPythonQt_Wrap_Qtgui:BOOL=ON
    -DPythonQt_Wrap_Qtuitools:BOOL=ON
  )

###############################################################################
# ctkPythonConsole

if(DD_QT_VERSION EQUAL 4)
  set(ctkPythonConsole_TAG 495ae0f)
else()
  set(ctkPythonConsole_TAG add-qt5-support)
endif()

ExternalProject_Add(ctkPythonConsole
  GIT_REPOSITORY https://github.com/patmarion/ctkPythonConsole
  GIT_TAG ${ctkPythonConsole_TAG}
  CMAKE_CACHE_ARGS
    ${default_cmake_args}
    ${qt_args}
    ${python_args}
  DEPENDS
    PythonQt
  )

###############################################################################
# QtPropertyBrowser

if(DD_QT_VERSION EQUAL 4)
  set(QtPropertyBrowser_TAG baf10af)
else()
  set(QtPropertyBrowser_TAG 72a0272)
endif()

ExternalProject_Add(QtPropertyBrowser
  GIT_REPOSITORY https://github.com/patmarion/QtPropertyBrowser
  GIT_TAG ${QtPropertyBrowser_TAG}
  CMAKE_CACHE_ARGS
    ${default_cmake_args}
    ${qt_args}
  )


###############################################################################
# vtk

if(USE_SYSTEM_VTK)

  if(APPLE)
    set(vtk_homebrew_dir /usr/local/opt/vtk7/lib/cmake/vtk-7.1)
  endif()

  find_package(VTK REQUIRED PATHS ${vtk_homebrew_dir})
  if (VTK_VERSION VERSION_LESS 6.2)
    message(FATAL_ERROR "Director requires VTK version 6.2 or greater."
      " System has VTK version ${VTK_VERSION}")
  endif()
  check_vtk_qt_version()

  set(vtk_args -DVTK_DIR:PATH=${VTK_DIR})

elseif(USE_PRECOMPILED_VTK)

  set(url_base "http://patmarion.com/bottles")

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

  if (ubuntu_version EQUAL 14.04)
    if(DD_QT_VERSION EQUAL 4)
      set(vtk_package_url ${url_base}/vtk7.1-qt4.8-python2.7-ubuntu14.04.tar.gz)
      set(vtk_package_md5 fe5c16f427a497b5713c52a68ecf564d)
    else()
      message(FATAL_ERROR "Compiling director with Qt5 is not supported on Ubuntu 14.04. "
               "Please set DD_QT_VERSION to 4.")
    endif()
  elseif(ubuntu_version EQUAL 16.04)
    if(DD_QT_VERSION EQUAL 4)
      set(vtk_package_url ${url_base}/vtk7.1-qt4.8-python2.7-ubuntu16.04.tar.gz)
      set(vtk_package_md5 1291e072405a3982b559ec011c3cf2a1)
    else()
      set(vtk_package_url ${url_base}/vtk7.1-qt5.5-python2.7-ubuntu16.04.tar.gz)
      set(vtk_package_md5 5ac930a7b1c083f975115d5970fb1a34)
    endif()
  else()
    message(FATAL_ERROR "USE_PRECOMPILED_VTK requires Ubuntu 14.04 or 16.04 "
            "but the detected system version does not match. "
            "Please disable USE_PRECOMPILED_VTK.")
  endif()

  ExternalProject_Add(vtk-precompiled
    URL ${vtk_package_url}
    URL_MD5 ${vtk_package_md5}
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND ${CMAKE_COMMAND} -E copy_directory
      ${source_prefix}/vtk-precompiled ${install_prefix}
  )

  set(vtk_args -DVTK_DIR:PATH=${install_prefix}/lib/cmake/vtk-7.1)
  set(vtk_depends vtk-precompiled)

else()

  ExternalProject_Add(vtk
    GIT_REPOSITORY git://vtk.org/VTK.git
    GIT_TAG v8.0.0
    CMAKE_CACHE_ARGS
      ${default_cmake_args}
      ${python_args}
      ${qt_args}
      -DBUILD_TESTING:BOOL=OFF
      -DBUILD_EXAMPLES:BOOL=OFF
      -DVTK_RENDERING_BACKEND:STRING=OpenGL2
      -DVTK_QT_VERSION:STRING=${DD_QT_VERSION}
      -DVTK_PYTHON_VERSION:STRING=2
      -DModule_vtkGUISupportQt:BOOL=ON
      -DVTK_WRAP_PYTHON:BOOL=ON
    )

  set(vtk_args -DVTK_DIR:PATH=${install_prefix}/lib/cmake/vtk-7.1)
  set(vtk_depends vtk)

endif()



###############################################################################
# pcl, flann

if(USE_PCL AND NOT USE_SYSTEM_PCL)

  # boost is an external dependency
  find_package(Boost REQUIRED)
  set(boost_args
    -DBoost_INCLUDE_DIR:PATH=${Boost_INCLUDE_DIR}
  )

  ExternalProject_Add(
    flann
    GIT_REPOSITORY http://github.com/mariusmuja/flann
    GIT_TAG cee08ec38a8df7bc70397f10a4d30b9b33518bb4
    CMAKE_CACHE_ARGS
      ${default_cmake_args}
      ${boost_args}
      ${python_args}
      -DBUILD_MATLAB_BINDINGS:BOOL=OFF
      -DBUILD_PYTHON_BINDINGS:BOOL=OFF
      -DBUILD_C_BINDINGS:BOOL=OFF
  )

  # flann used to install to lib64, but it seems that it doesn't do that anymore...
  if(FALSE AND NOT APPLE AND ${CMAKE_SYSTEM_PROCESSOR} STREQUAL x86_64)
    set(flann_lib_dir lib64)
  else()
    set(flann_lib_dir lib)
  endif()

  set(so_extension so)
  if(APPLE)
    set(so_extension dylib)
  endif()

  set(flann_args
    -DFLANN_INCLUDE_DIR:PATH=${install_prefix}/include
    -DFLANN_INCLUDE_DIRS:PATH=${install_prefix}/include
    -DFLANN_LIBRARY:PATH=${install_prefix}/${flann_lib_dir}/libflann_cpp.${so_extension}
    -DFLANN_LIBRARY_DEBUG:PATH=${install_prefix}/${flann_lib_dir}/libflann_cpp-gd.${so_extension}
    )

  # Requires build-in suffixes. Otherwise, will get the following error:
  # error: unable to find numeric literal operator ‘operator""Q’
  if(CMAKE_COMPILER_IS_GNUCXX)
    set(gxx_extra_flags -fext-numeric-literals)
  endif()

  ExternalProject_Add(
    pcl
    GIT_REPOSITORY http://github.com/pointcloudlibrary/pcl.git
    GIT_TAG pcl-1.8.0
    CMAKE_CACHE_ARGS
      ${default_cmake_args}
      ${eigen_args}
      ${boost_args}
      ${flann_args}
      ${vtk_args}
      -DPCL_SHARED_LIBS:BOOL=${BUILD_SHARED_LIBS}
      -DBUILD_TESTS:BOOL=OFF
      -DBUILD_global_tests:BOOL=OFF
      -DBUILD_examples:BOOL=OFF
      -DBUILD_tools:BOOL=OFF
      -DBUILD_apps:BOOL=OFF
      -DBUILD_visualization:BOOL=OFF
      "-DCMAKE_CXX_FLAGS:STRING=-std=c++11 ${gxx_extra_flags}"

    DEPENDS
      ${vtk_depends}
      ${eigen_depends}
      flann
  )

  set(pcl_depends pcl)

endif()


###############################################################################
# PointCloudLibraryPlugin

if(USE_PCL)

ExternalProject_Add(PointCloudLibraryPlugin
  GIT_REPOSITORY https://github.com/patmarion/PointCloudLibraryPlugin.git
  GIT_TAG 84de916
  CMAKE_CACHE_ARGS
    ${default_cmake_args}
    ${eigen_args}
    ${boost_args}
    ${flann_args}
    ${python_args}
    ${vtk_args}
    -DPCL_REQUIRED_VERSION:STRING=1.8.0
  DEPENDS
    ${pcl_depends}
    ${vtk_depends}
  )

endif()


###############################################################################
# camera driver

if(USE_OPENNI2_LCM)

  ExternalProject_Add(openni2-camera-lcm
    GIT_REPOSITORY https://github.com/openhumanoids/openni2-camera-lcm
    GIT_TAG 6bd5573
    CMAKE_CACHE_ARGS
      ${default_cmake_args}
      -DINSTALL_BOT_SPY:BOOL=OFF
    DEPENDS
      ${lcm_depends}
    )

endif()

if(USE_PERCEPTION)

  ExternalProject_Add(cv-utils
    GIT_REPOSITORY https://github.com/patmarion/cv-utils.git
    GIT_TAG c4939fedf66c767de15607adde3aff44ab2b503b
    CMAKE_CACHE_ARGS
      ${default_cmake_args}
    DEPENDS
      ${lcm_depends} ${pcl_depends}
    )

  set(cvutils_depends cv-utils)

endif()


###############################################################################
# apriltags

if(USE_APRILTAGS)

  ExternalProject_Add(apriltags
    GIT_REPOSITORY https://github.com/psiorx/apriltags-pod.git
    GIT_TAG 8534e91
    CMAKE_CACHE_ARGS
      ${default_cmake_args}
    )

  ExternalProject_Add(apriltags_driver
    GIT_REPOSITORY https://github.com/patmarion/apriltags_driver.git
    GIT_TAG fb0eff3
    CMAKE_CACHE_ARGS
      ${default_cmake_args}
    DEPENDS
      ${lcm_depends} apriltags
    )

endif()


###############################################################################
# director

ExternalProject_Add(director
  SOURCE_DIR ${Superbuild_SOURCE_DIR}/../..
  DOWNLOAD_COMMAND ""
  CMAKE_CACHE_ARGS

    -DUSE_LCM:BOOL=${USE_LCM}
    -DUSE_LCMGL:BOOL=${USE_LCMGL}
    -DUSE_OCTOMAP:BOOL=${USE_OCTOMAP}
    -DUSE_COLLECTIONS:BOOL=${USE_COLLECTIONS}
    -DUSE_LIBBOT:BOOL=${USE_LIBBOT}
    -DUSE_DRAKE:BOOL=${USE_DRAKE}
    -DDD_QT_VERSION:STRING=${DD_QT_VERSION}
    -DUSE_PERCEPTION:BOOL=${USE_PERCEPTION}
    ${default_cmake_args}
    ${eigen_args}
    ${boost_args}
    ${flann_args}
    ${vtk_args}
    ${python_args}
    ${qt_args}

  DEPENDS

    ${vtk_depends}
    ${pcl_depends}
    ${eigen_depends}
    ${lcm_depends}
    ${libbot_depends}
    ${cvutils_depends}
    PythonQt
    ctkPythonConsole
    QtPropertyBrowser

  )
