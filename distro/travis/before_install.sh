#!/bin/bash

set -xe

scriptDir=$(cd $(dirname $0) && pwd)


make_vtk_homebrew_bottle()
{
  brew tap patmarion/director

  # old options: --with-qt --without-boost --without-pyqt --without-sip
  # not needed with new vtk5.rb from above tap

  $scriptDir/brew_install.sh vtk5 --build-bottle
  brew bottle vtk5

  $scriptDir/copy_files.sh vtk5*.tar.gz
}

install_vtk_homebrew_bottle()
{
  brew tap patmarion/director
  brew install vtk5
}

if [ "$TRAVIS_OS_NAME" = "linux" ]; then
	sudo apt-get update -qq
  sudo apt-get install -y build-essential cmake libqt4-dev libvtk5-dev libvtk5-qt4-dev libvtk-java python-dev python-vtk python-numpy xvfb

  # start Xvfb for DISPLAY=:99.0
  /sbin/start-stop-daemon --start --quiet --pidfile /tmp/custom_xvfb_99.pid --make-pidfile \
                          --background --exec /usr/bin/Xvfb -- :99 -ac -screen 0 1280x1024x16

elif [ "$TRAVIS_OS_NAME" = "osx" ]; then

  brew tap homebrew/python
  brew tap homebrew/science
  brew update > brew_update_log.txt
  #brew upgrade

  brew install python qt
  brew install numpy || echo "error on brew install numpy"

  #install_vtk_homebrew_bottle
  make_vtk_homebrew_bottle
fi
