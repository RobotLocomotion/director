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
  osid=$( cat /etc/os-release | grep "^ID=" | sed s/ID=// )
  osversion=$( cat /etc/os-release | grep "VERSION_ID=" | sed s/VERSION_ID=// | sed 's/"\(.*\)"/\1/' )
  site_name=docker-$osid-$osversion
  USE_SYSTEM_VTK=OFF
  nproc=$(nproc)
elif [ "$TRAVIS_OS_NAME" = "osx" ]; then
  osx_version=$(sw_vers -productVersion)
  xcode_version=$(xcodebuild -version | grep Xcode | sed s/Xcode\ //)
  site_name=osx-$osx_version-xcode-$xcode_version
  USE_SYSTEM_VTK=ON
  nproc=2
fi

# build
cd $TRAVIS_BUILD_DIR
mkdir build && cd build
cmake -DUSE_LCM=$USE_LCM -DUSE_LIBBOT=$USE_LIBBOT -DUSE_LCMGL=$USE_LCMGL -DUSE_SYSTEM_VTK=$USE_SYSTEM_VTK ../distro/superbuild
make -j $nproc

# test
cd $TRAVIS_BUILD_DIR/build/src/director-build
cmake_command=$(grep CMAKE_COMMAND CMakeCache.txt | cut -d = -f 2)
$cmake_command -DSITE:STRING=${site_name} -DBUILDNAME:STRING=${build_name} .
ctest -j 1 --dashboard Experimental --track travis --output-on-failure
