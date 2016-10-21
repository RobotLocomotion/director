#!/bin/bash

set -xe

scriptDir=$(cd $(dirname $0) && pwd)


if [ "$TRAVIS_PULL_REQUEST" = "false" ]; then
  cd $TRAVIS_BUILD_DIR
  build_name=$(git rev-parse --short HEAD)-$TRAVIS_BRANCH
else
  build_name=${TRAVIS_PULL_REQUEST_SHA:0:7}-${TRAVIS_PULL_REQUEST_BRANCH}
fi


# build
cd $TRAVIS_BUILD_DIR
mkdir build && cd build
cmake -DUSE_LCM:BOOL=$USE_LCM ../distro/superbuild
make -j2

# test
cd $TRAVIS_BUILD_DIR/build/src/director-build
cmake -DSITE:STRING=travis-$TRAVIS_OS_NAME -DBUILDNAME:STRING=${build_name}_lcm-$USE_LCM .
ctest -j 1 --dashboard Experimental --track travis --output-on-failure
