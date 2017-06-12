#!/bin/bash
set -xe

scriptDir=$(cd $(dirname $0) && pwd)


$scriptDir/install_deps.sh

if [ "$TRAVIS_OS_NAME" = "linux" ]; then
  # Xvfb needs to be running during testing and packaging
  Xvfb :99 -ac -screen 0 1280x1024x16 &
fi

$scriptDir/build_and_test.sh
$scriptDir/docs_and_package.sh
