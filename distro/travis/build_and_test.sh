#!/bin/bash

set -xe

scriptDir=$(cd $(dirname $0) && pwd)


if [ "$TRAVIS_PULL_REQUEST" = "false" ]; then
  cd $TRAVIS_BUILD_DIR
  build_name=$(git rev-parse --short HEAD)-$TRAVIS_BRANCH
else
  build_name=${TRAVIS_PULL_REQUEST_SHA:0:7}-${TRAVIS_PULL_REQUEST_BRANCH}
fi

if [ "$USE_LCM" = "ON" ]; then
  build_name=${build_name}_lcm
fi

if [ "$USE_LIBBOT" = "ON" ]; then
  build_name=${build_name}_libbot
fi

if [[ "$TRAVIS_OS_NAME" == "linux" ]]; then
  qt_version=4
  system_vtk=OFF
  download_vtk_package=ON
else
  qt_version=5
  system_vtk=ON
  download_vtk_package=OFF
  export CMAKE_MODULE_PATH=/usr/local/opt/qt/lib/cmake/:/usr/local/opt/vtk@8.0/lib/cmake/vtk-7.1
fi

# build
cd $TRAVIS_BUILD_DIR
mkdir build && cd build
cmake \
  -DDD_QT_VERSION:STRING=${qt_version} \
  -DUSE_SYSTEM_VTK:BOOL=${system_vtk} \
  -DDOWNLOAD_VTK_PACKAGE:BOOL=${download_vtk_package} \
  -DUSE_LCM:BOOL=$USE_LCM \
  -DUSE_LCMGL:BOOL=$USE_LCMGL \
  -DUSE_LIBBOT:BOOL=$USE_LIBBOT \
  ../distro/superbuild
make -j2

# test
cd $TRAVIS_BUILD_DIR/build/src/director-build
cmake -DSITE:STRING=travis-$TRAVIS_OS_NAME -DBUILDNAME:STRING=${build_name} .
ctest -j 1 --dashboard Experimental --track travis --output-on-failure
