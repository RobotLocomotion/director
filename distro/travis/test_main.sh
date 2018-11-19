#!/bin/bash
set -xe

scriptDir=$(cd $(dirname $0) && pwd)

if [ "$TRAVIS_OS_NAME" = "osx" ]; then
  export PATH=/usr/local/opt/python/libexec/bin:$PATH
fi

$scriptDir/install_deps.sh

if [ "$TRAVIS_OS_NAME" = "linux" ]; then
  # Xvfb needs to be running during testing and packaging
  /sbin/start-stop-daemon --start --quiet --pidfile /tmp/custom_xvfb_99.pid --make-pidfile \
                          --background --exec /usr/bin/Xvfb -- :99 -ac -screen 0 1280x1024x16
fi

$scriptDir/build_and_test.sh

export LD_LIBRARY_PATH=$TRAVIS_BUILD_DIR/build/install/lib

$scriptDir/docs_and_package.sh
