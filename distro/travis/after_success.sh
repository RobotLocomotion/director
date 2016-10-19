#!/bin/bash

set -xe

scriptDir=$(cd $(dirname $0) && pwd)

upload_files()
{
  echo "upload files"
}

install_patchelf()
{
  wget http://nixos.org/releases/patchelf/patchelf-0.8/patchelf-0.8.tar.gz
  tar -zxf patchelf-0.8.tar.gz
  pushd  patchelf-0.8
  ./configure --prefix=$PWD/../patchelf-install
  make install
  popd
}

cd $TRAVIS_BUILD_DIR/build/src/director-build
make python-coverage
make docs-doxygen
make docs-sphinx-generate
make docs-sphinx-build

mv $TRAVIS_BUILD_DIR/docs/sphinx/_build/html sphinx_docs
mv $TRAVIS_BUILD_DIR/build/src/director-build/docs/doxygen/html doxygen_docs
mv $TRAVIS_BUILD_DIR/build/src/director-build/python-coverage/html python_coverage

tar -czf sphinx_docs.tar.gz sphinx_docs
tar -czf doxygen_docs.tar.gz doxygen_docs
tar -czf python_coverage.tar.gz python_coverage


if [ "$TRAVIS_OS_NAME" = "linux" ]; then

  cd $TRAVIS_BUILD_DIR/distro/package
  install_patchelf
  python fixup_elf.py $TRAVIS_BUILD_DIR/build/install $TRAVIS_BUILD_DIR/build/install/lib $TRAVIS_BUILD_DIR/distro/package/patchelf-install/bin/patchelf

elif [ "$TRAVIS_OS_NAME" = "osx" ]; then

  cd $TRAVIS_BUILD_DIR/distro/package
  ./make_app_bundle.sh
  find . -name \*.h DirectorConsole.app | xargs ls
  tar -czf DirectorConsole.tar.gz DirectorConsole.app
  ls -alh DirectorConsole.tar.gz
  #$scriptDir/copy_files.sh DirectorConsole.tar.gz

fi




if [ "${TRAVIS_PULL_REQUEST}" = "false" ]; then
  upload_files
fi
