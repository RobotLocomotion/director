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

if [ "$TRAVIS_OS_NAME" = "linux" ]; then

  ubuntu_version=$(lsb_release --release --short)
  site_name=docker-ubuntu-$ubuntu_version
  USE_SYSTEM_VTK=OFF
  nproc=$(nproc)
  if [ "$ubuntu_version" = "18.04" ]; then
    qt_version=5
  else
    qt_version=4
  fi

elif [ "$TRAVIS_OS_NAME" = "osx" ]; then
  osx_version=$(sw_vers -productVersion)
  xcode_version=$(xcodebuild -version | grep Xcode | sed s/Xcode\ //)
  site_name=osx-$osx_version-xcode-$xcode_version
  USE_SYSTEM_VTK=ON
  qt_version=5
  nproc=2
fi

# build
cd $TRAVIS_BUILD_DIR
mkdir build && cd build
cmake -DUSE_LCM=$USE_LCM -DUSE_LIBBOT=$USE_LIBBOT -DUSE_LCMGL=$USE_LCMGL -DUSE_SYSTEM_VTK=$USE_SYSTEM_VTK -DDD_QT_VERSION=$qt_version ../distro/superbuild
make -j $nproc

# test
cd $TRAVIS_BUILD_DIR/build/src/director-build
cmake_command=$(grep CMAKE_COMMAND CMakeCache.txt | cut -d = -f 2)
$cmake_command -DSITE:STRING=${site_name} -DBUILDNAME:STRING=${build_name} .

# TODO
# this was added for ubuntu18 on travis-ci but shouldn't be required, need to debug.
export LD_LIBRARY_PATH=$TRAVIS_BUILD_DIR/build/install/lib

ctest -j 1 --dashboard Experimental --track travis --output-on-failure -V
