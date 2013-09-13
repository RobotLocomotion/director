project(VES_SUPERBUILD NONE)

include(ExternalProject)

option(VES_USE_LIBARCHIVE "Should VES include libarchive support" OFF)
option(VES_USE_CURL "Should VES include cURL support" OFF)
option(VES_USE_PCL "Should VES include PCL support" OFF)
set(PCL_SUPERBUILD_DIR "PCL_SUPERBUILD_DIR-NOTFOUND" CACHE PATH
  "Directory containing CMakeExternals/Install with PCL and its dependencies")
if (VES_USE_PCL)
  mark_as_advanced(CLEAR PCL_SUPERBUILD_DIR)
else()
  mark_as_advanced(FORCE PCL_SUPERBUILD_DIR)
endif()

option(VES_HOST_SUPERBUILD "Build VES and dependent subprojects for host architecture" OFF)
option(VES_ANDROID_SUPERBUILD "Build VES and dependent subprojects for Android" OFF)
option(VES_IOS_SUPERBUILD "Build VES and dependent subprojects for iOS" OFF)
set(VES_DOWNLOAD_PREFIX "${CMAKE_BINARY_DIR}/downloads" CACHE PATH
  "Directory to store and extract tarballs of external dependencies")

set(base "${CMAKE_BINARY_DIR}/CMakeExternals")
set_property(DIRECTORY PROPERTY EP_BASE ${base})

# set a default build type if it is undefined, then make sure it goes in the cache
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "Build configuration type" FORCE)
set(build_type ${CMAKE_BUILD_TYPE})

set(source_prefix ${base}/Source)
set(build_prefix ${base}/Build)
set(install_prefix ${base}/Install)

set(toolchain_dir "${CMAKE_CURRENT_SOURCE_DIR}/CMake/toolchains")
set(ves_src_dir "${CMAKE_CURRENT_SOURCE_DIR}")
set(ves_patch_dir "${ves_src_dir}/CMake/patches")
set(vtk_src_dir "${source_prefix}/vtk")
set(vtk_patch_file ${CMAKE_BINARY_DIR}/vtk-patch.cmake)
configure_file(${CMAKE_SOURCE_DIR}/CMake/vtk-patch.cmake.in
               ${vtk_patch_file} @ONLY)

find_package(PythonInterp REQUIRED)
find_package(Git REQUIRED)

set(module_defaults
  -DVTK_Group_StandAlone:BOOL=OFF
  -DVTK_Group_Rendering:BOOL=OFF
  -DModule_vtkFiltersCore:BOOL=ON
  -DModule_vtkFiltersModeling:BOOL=ON
  -DModule_vtkFiltersSources:BOOL=ON
  -DModule_vtkFiltersGeometry:BOOL=ON
  -DModule_vtkIOGeometry:BOOL=ON
  -DModule_vtkIOLegacy:BOOL=ON
  -DModule_vtkIOXML:BOOL=ON
  -DModule_vtkIOImage:BOOL=ON
  -DModule_vtkIOPLY:BOOL=ON
  -DModule_vtkIOInfovis:BOOL=ON
  -DModule_vtkImagingCore:BOOL=ON
  -DModule_vtkParallelCore:BOOL=ON
  -DModule_vtkRenderingCore:BOOL=ON
  -DModule_vtkRenderingFreeType:BOOL=ON
)


macro(force_build proj)
  ExternalProject_Add_Step(${proj} forcebuild
    COMMAND ${CMAKE_COMMAND} -E remove ${base}/Stamp/${proj}/${proj}-build
    DEPENDEES configure
    DEPENDERS build
    ALWAYS 1
  )
endmacro()


macro(install_eigen)
  set(eigen_url http://vtk.org/files/support/eigen-3.1.2.tar.gz)
  set(eigen_md5 bb639388192cb80f1ee797f5dbdbe74f)
  ExternalProject_Add(
    eigen
    SOURCE_DIR ${source_prefix}/eigen
    DOWNLOAD_DIR ${VES_DOWNLOAD_PREFIX}
    URL ${eigen_url}
    URL_MD5 ${eigen_md5}
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND ${CMAKE_COMMAND} -E copy_directory "${source_prefix}/eigen/Eigen" "${install_prefix}/eigen/Eigen"
                 && ${CMAKE_COMMAND} -E copy_directory "${source_prefix}/eigen/unsupported" "${install_prefix}/eigen/unsupported"
  )
endmacro()


#
# libarchive download
#
macro(download_libarchive)
  ExternalProject_Add(
    libarchive-download
    SOURCE_DIR ${source_prefix}/libarchive
    GIT_REPOSITORY git://github.com/libarchive/libarchive.git
    GIT_TAG v3.0.4
    PATCH_COMMAND git apply ${ves_src_dir}/CMake/libarchive.patch
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND ""
  )
endmacro()


#
# libarchive compile
#
macro(compile_libarchive tag)
  set(proj libarchive-${tag})
  ExternalProject_Add(
    ${proj}
    SOURCE_DIR ${source_prefix}/libarchive
    DOWNLOAD_COMMAND ""
    DEPENDS libarchive-download
    CMAKE_ARGS
      -DCMAKE_INSTALL_PREFIX:PATH=${install_prefix}/${proj}
      -DCMAKE_BUILD_TYPE:STRING=${build_type}
      -DBUILD_SHARED_LIBS:BOOL=OFF
      -DENABLE_NETTLE:BOOL=OFF
      -DENABLE_OPENSSL:BOOL=OFF
      -DENABLE_TAR:BOOL=OFF
      -DENABLE_TAR_SHARED:BOOL=OFF
      -DENABLE_CPIO:BOOL=OFF
      -DENABLE_CPIO_SHARED:BOOL=OFF
      -DENABLE_XATTR:BOOL=OFF
      -DENABLE_ACL:BOOL=OFF
      -DENABLE_ICONV:BOOL=OFF
      -DENABLE_TEST:BOOL=OFF
  )

  force_build(${proj})
endmacro()


#
# libarchive crosscompile
#
macro(crosscompile_libarchive proj toolchain_file)
  ExternalProject_Add(
    ${proj}
    SOURCE_DIR ${source_prefix}/libarchive
    DOWNLOAD_COMMAND ""
    DEPENDS libarchive-download
    CMAKE_ARGS
      -DCMAKE_TOOLCHAIN_FILE:FILEPATH=${toolchain_dir}/${toolchain_file}
      -DCMAKE_INSTALL_PREFIX:PATH=${install_prefix}/${proj}
      -DCMAKE_BUILD_TYPE:STRING=${build_type}
      -DBUILD_SHARED_LIBS:BOOL=OFF
      -DENABLE_NETTLE:BOOL=OFF
      -DENABLE_OPENSSL:BOOL=OFF
      -DENABLE_TAR:BOOL=OFF
      -DENABLE_TAR_SHARED:BOOL=OFF
      -DENABLE_CPIO:BOOL=OFF
      -DENABLE_CPIO_SHARED:BOOL=OFF
      -DENABLE_XATTR:BOOL=OFF
      -DENABLE_ACL:BOOL=OFF
      -DENABLE_ICONV:BOOL=OFF
      -DENABLE_TEST:BOOL=OFF
  )

  force_build(${proj})
endmacro()


macro(download_curl)
  ExternalProject_Add(
    curl-download
    GIT_REPOSITORY git://github.com/patmarion/curl.git
    GIT_TAG origin/v7.24.0-with-cmake-patches
    SOURCE_DIR ${source_prefix}/curl
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND ""
  )
endmacro()


macro(compile_curl tag)
  set(proj curl-${tag})
  ExternalProject_Add(
    ${proj}
    DOWNLOAD_COMMAND ""
    SOURCE_DIR ${source_prefix}/curl
    DEPENDS curl-download
    CMAKE_ARGS
      -DCMAKE_INSTALL_PREFIX:PATH=${install_prefix}/${proj}
      -DCMAKE_BUILD_TYPE:STRING=${build_type}
      -DBUILD_CURL_EXE:BOOL=OFF
      -DBUILD_CURL_TESTS:BOOL=OFF
      -DCURL_STATICLIB:BOOL=ON
  )
endmacro()


macro(crosscompile_curl proj toolchain_file)
  ExternalProject_Add(
    ${proj}
    DOWNLOAD_COMMAND ""
    SOURCE_DIR ${source_prefix}/curl
    DEPENDS curl-download
    CMAKE_ARGS
      -DCMAKE_INSTALL_PREFIX:PATH=${install_prefix}/${proj}
      -DCMAKE_BUILD_TYPE:STRING=${build_type}
      -DCURL_STATICLIB:BOOL=ON
      -DBUILD_CURL_EXE:BOOL=OFF
      -DBUILD_CURL_TESTS:BOOL=OFF
      -DCMAKE_TOOLCHAIN_FILE:FILEPATH=${toolchain_dir}/${toolchain_file}
      -C ${toolchain_dir}/curl-TryRunResults.cmake
  )
endmacro()


macro(compile_vtk proj)
  if(NOT VES_HOST_SUPERBUILD)
    set(makecmd make)
    if(CMAKE_GENERATOR MATCHES "NMake Makefiles")
      set(makecmd nmake)
    elseif(CMAKE_GENERATOR MATCHES "Ninja")
      set(makecmd ninja)
    endif()
    set(vtk_host_build_command BUILD_COMMAND ${makecmd} vtkCompileTools)
  endif()
  ExternalProject_Add(
    ${proj}
    SOURCE_DIR ${vtk_src_dir}
    DOWNLOAD_DIR ${VES_DOWNLOAD_PREFIX}
    URL http://www.vtk.org/files/release/6.0/vtk-6.0.0.tar.gz
    URL_MD5 72ede4812c90bdc55172702f0cad02bb
    PATCH_COMMAND ${CMAKE_COMMAND} -P ${vtk_patch_file}
    INSTALL_COMMAND ""
    ${vtk_host_build_command}
    CMAKE_ARGS
      -DCMAKE_INSTALL_PREFIX:PATH=${install_prefix}/${proj}
      -DCMAKE_BUILD_TYPE:STRING=${build_type}
      -DBUILD_SHARED_LIBS:BOOL=ON
      -DBUILD_TESTING:BOOL=OFF
      ${module_defaults}
  )
endmacro()


macro(crosscompile_vtk proj toolchain_file)
  ExternalProject_Add(
    ${proj}
    SOURCE_DIR ${vtk_src_dir}
    DOWNLOAD_COMMAND ""
    DEPENDS vtk-host
    CMAKE_ARGS
      -DCMAKE_INSTALL_PREFIX:PATH=${install_prefix}/${proj}
      -DCMAKE_BUILD_TYPE:STRING=${build_type}
      -DBUILD_SHARED_LIBS:BOOL=OFF
      -DBUILD_TESTING:BOOL=OFF
      -DCMAKE_TOOLCHAIN_FILE:FILEPATH=${toolchain_dir}/${toolchain_file}
      -DVTKCompileTools_DIR:PATH=${build_prefix}/vtk-host
      ${module_defaults}
      -C ${toolchain_dir}/TryRunResults.cmake
  )
endmacro()


macro(compile_ves proj)
  set(tag host)
  list(APPEND VES_SUPERBUILD_${tag}_OPTS
    "-DVES_USE_CURL:BOOL=${VES_USE_CURL}";
    "-DVES_USE_LIBARCHIVE:BOOL=${VES_USE_LIBARCHIVE}";
    "-DVES_USE_PCL:BOOL=${VES_USE_PCL}"
    )
  if (VES_USE_CURL)
    list(APPEND VES_SUPERBUILD_${tag}_DEPS curl-${tag})
  endif()
  if (VES_USE_LIBARCHIVE)
    list(APPEND VES_SUPERBUILD_${tag}_OPTS
      "-DLibArchive_DIR:PATH=${install_prefix}/libarchive-${tag}/lib/LibArchive/cmake";
      )
    list(APPEND VES_SUPERBUILD_${tag}_DEPS libarchive-${tag})
  endif()
  if (VES_USE_PCL)
    list(APPEND VES_SUPERBUILD_${tag}_OPTS
      "-DPCL_SUPERBUILD_DIR:PATH=${PCL_SUPERBUILD_DIR}";
      "-DPCL_BUILD_TARGET:STRING=${tag}"
      )
  endif()
  ExternalProject_Add(
    ${proj}
    SOURCE_DIR ${ves_src_dir}
    DOWNLOAD_COMMAND ""
    DEPENDS vtk-${tag} eigen ${VES_SUPERBUILD_${tag}_DEPS}
    CMAKE_ARGS
      -DCMAKE_INSTALL_PREFIX:PATH=${install_prefix}/${proj}
      -DCMAKE_BUILD_TYPE:STRING=${build_type}
      -DBUILD_TESTING:BOOL=ON
      -DBUILD_SHARED_LIBS:BOOL=OFF
      ${VES_SUPERBUILD_${tag}_OPTS}
      -DVES_USE_VTK:BOOL=ON
      -DVES_NO_SUPERBUILD:BOOL=ON
      -DVES_USE_DESKTOP_GL:BOOL=ON
      -DVTK_DIR:PATH=${build_prefix}/vtk-${tag}
      -DEIGEN_INCLUDE_DIR:PATH=${install_prefix}/eigen
      -DCMAKE_CXX_FLAGS:STRING=${VES_CXX_FLAGS} -fPIC
  )

  force_build(${proj})
endmacro()


macro(crosscompile_ves proj tag toolchain_file)
  list(APPEND VES_SUPERBUILD_${tag}_OPTS
    "-DVES_USE_CURL:BOOL=${VES_USE_CURL}";
    "-DVES_USE_LIBARCHIVE:BOOL=${VES_USE_LIBARCHIVE}"
    "-DVES_USE_PCL:BOOL=${VES_USE_PCL}"
    )
  if (VES_USE_CURL)
    list(APPEND VES_SUPERBUILD_${tag}_OPTS
      "-DCURL_INCLUDE_DIR:PATH=${install_prefix}/curl-${tag}/include";
      "-DCURL_LIBRARY:PATH=${install_prefix}/curl-${tag}/lib/libcurl.a"
      )
    list(APPEND VES_SUPERBUILD_${tag}_DEPS curl-${tag})
  endif()
  if (VES_USE_LIBARCHIVE)
    list(APPEND VES_SUPERBUILD_${tag}_OPTS
      "-DLibArchive_DIR:PATH=${install_prefix}/libarchive-${tag}/lib/LibArchive/cmake";
      )
    list(APPEND VES_SUPERBUILD_${tag}_DEPS libarchive-${tag})
  endif()
  if (VES_USE_PCL)
    list(APPEND VES_SUPERBUILD_${tag}_OPTS
      "-DPCL_SUPERBUILD_DIR:PATH=${PCL_SUPERBUILD_DIR}";
      "-DPCL_BUILD_TARGET:STRING=${tag}"
      )
  endif()
  ExternalProject_Add(
    ${proj}
    SOURCE_DIR ${ves_src_dir}
    DOWNLOAD_COMMAND ""
    DEPENDS vtk-${tag} eigen ${VES_SUPERBUILD_${tag}_DEPS}
    CMAKE_ARGS
      -DCMAKE_INSTALL_PREFIX:PATH=${install_prefix}/${proj}
      -DCMAKE_BUILD_TYPE:STRING=${build_type}
      -DCMAKE_TOOLCHAIN_FILE:FILEPATH=${toolchain_dir}/${toolchain_file}
      -DCMAKE_CXX_FLAGS:STRING=${VES_CXX_FLAGS}
      -DBUILD_SHARED_LIBS:BOOL=OFF
      -DVES_USE_VTK:BOOL=ON
      -DVES_NO_SUPERBUILD:BOOL=ON
      -DVTK_DIR:PATH=${build_prefix}/vtk-${tag}
      ${VES_SUPERBUILD_${tag}_OPTS}
      -DEIGEN_INCLUDE_DIR:PATH=${install_prefix}/eigen
      -DPYTHON_EXECUTABLE:FILEPATH=${PYTHON_EXECUTABLE}
  )

  force_build(${proj})
endmacro()


install_eigen()
if (VES_USE_CURL)
  download_curl()
endif()
if (VES_USE_LIBARCHIVE)
  download_libarchive()
endif()
compile_vtk(vtk-host)

if(VES_IOS_SUPERBUILD)
  foreach(target ios-simulator ios-device)
    if (VES_USE_CURL)
      crosscompile_curl(curl-${target} toolchain-${target}.cmake)
    endif()
    if (VES_USE_LIBARCHIVE)
      crosscompile_libarchive(libarchive-${target} toolchain-${target}.cmake)
    endif()
    crosscompile_vtk(vtk-${target} toolchain-${target}.cmake)
    crosscompile_ves(ves-${target} ${target} toolchain-${target}.cmake)
  endforeach()
endif()

if(VES_ANDROID_SUPERBUILD)
  if (VES_USE_CURL)
    crosscompile_curl(curl-${target} toolchain-${target}.cmake)
  endif()
  if (VES_USE_LIBARCHIVE)
    crosscompile_libarchive(libarchive-android android.toolchain.cmake)
  endif()
  crosscompile_vtk(vtk-android android.toolchain.cmake)
  crosscompile_ves(ves-android android android.toolchain.cmake)
endif()

if(VES_HOST_SUPERBUILD)
  if (VES_USE_CURL)
    compile_curl(host)
  endif()
  if (VES_USE_LIBARCHIVE)
    compile_libarchive(host)
  endif()
  compile_ves(ves-host)
endif()

# create frameworks for OSX
if(VES_IOS_SUPERBUILD)
    add_custom_target(kiwi-framework ALL
      COMMAND ${CMAKE_SOURCE_DIR}/CMake/makeFramework.sh kiwi
      WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
      COMMENT "Creating kiwi framework")
    add_dependencies(kiwi-framework ves-ios-simulator ves-ios-device eigen)

    add_custom_target(vtk-framework ALL
      COMMAND ${CMAKE_SOURCE_DIR}/CMake/makeFramework.sh vtk
      WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
      COMMENT "Creating vtk framework")
    add_dependencies(vtk-framework vtk-ios-simulator vtk-ios-device)
endif()


# CTestCustom.cmake needs to be placed at the top level build directory
configure_file(${CMAKE_SOURCE_DIR}/CMake/CTestCustom.cmake.in
               ${CMAKE_BINARY_DIR}/CTestCustom.cmake COPYONLY)
