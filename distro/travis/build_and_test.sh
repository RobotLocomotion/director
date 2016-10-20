#!/bin/bash

set -xe

scriptDir=$(cd $(dirname $0) && pwd)

# build
cd $TRAVIS_BUILD_DIR
mkdir build && cd build
cmake -DUSE_LCM:BOOL=$USE_LCM ../distro/superbuild
make -j2

# test
cd $TRAVIS_BUILD_DIR/build/src/director-build
cmake -DSITE:STRING=travis-$TRAVIS_OS_NAME -DBUILDNAME:STRING=$TRAVIS_BRANCH_lcm-$USE_LCM .
ctest -j 1 --dashboard Experimental --track travis --output-on-failure
