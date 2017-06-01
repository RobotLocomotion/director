#!/bin/bash

set -xe

scriptDir=$(cd $(dirname $0) && pwd)


fetch_vtk_trusty()
{
  pushd /tmp
  curl -O https://d2mbb5ninhlpdu.cloudfront.net/vtk/vtk-v7.1.1-1584-g28deb56-qt-4.8.6-trusty-x86_64.tar.gz
  popd
  pushd /usr
  sudo tar -xvzf /tmp/vtk-v7.1.1-1584-g28deb56-qt-4.8.6-trusty-x86_64.tar.gz
  popd
}

if [ "$TRAVIS_OS_NAME" = "linux" ]; then
  sudo apt-get update -qq
  sudo apt-get install -y build-essential cmake curl \
    python-dev python-numpy python-yaml python-lxml xvfb \
    doxygen graphviz python-sphinx python-coverage \
    qt4-default libqt4-dev libqt4-declarative \
    libqt4-private-dev libfreetype6-dev libxt-dev libxml2-dev \
    libexpat-dev libjpeg-dev libtiff5-dev libglib2.0-dev

  sudo pip install --upgrade sphinx_rtd_theme breathe

  fetch_vtk_trusty

  # start Xvfb for DISPLAY=:99.0
  /sbin/start-stop-daemon --start --quiet --pidfile /tmp/custom_xvfb_99.pid --make-pidfile \
                          --background --exec /usr/bin/Xvfb -- :99 -ac -screen 0 1280x1024x16

elif [ "$TRAVIS_OS_NAME" = "osx" ]; then

  brew tap homebrew/python
  brew tap homebrew/science
  brew tap robotlocomotion/director
  brew tap-pin robotlocomotion/director

  brew update > brew_update_log.txt

  brew install qt vtk@8.0
  brew install doxygen graphviz
  brew install glib # for lcm
  brew ls --versions python || brew install python
  brew ls --versions numpy || brew install numpy || echo "error on brew install numpy"

  pip install pyyaml lxml Sphinx sphinx-rtd-theme coverage

fi
