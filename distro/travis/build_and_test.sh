#!/bin/bash

set -xe

scriptDir=$(cd $(dirname $0) && pwd)


# build
cd $TRAVIS_BUILD_DIR
make superbuild -j2

# test
cd build/src/director-build
cmake -DSITE:STRING=travis-$TRAVIS_OS_NAME -DBUILDNAME:STRING=$TRAVIS_OS_NAME-branch-$TRAVIS_BRANCH .
ctest -j 1 -L core --dashboard Experimental --track travis --output-on-failure
