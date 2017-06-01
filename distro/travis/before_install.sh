#!/bin/bash

set -xe

scriptDir=$(cd $(dirname $0) && pwd)


# Update CMake on trusty to meet minimum requirement for VTK (CMake-3.3)
update_cmake_trusty()
{
  wget https://cmake.org/files/v3.8/cmake-3.8.2-Linux-x86_64.tar.gz -O /tmp/cmake-3.8.2-Linux-x86_64.tar.gz 
  pushd /usr
  sudo tar -xvzf /tmp/cmake-3.8.2-Linux-x86_64.tar.gz --strip-components=1
  popd
}


fetch_vtk_trusty()
{
  wget https://d2mbb5ninhlpdu.cloudfront.net/vtk/vtk-v7.1.1-1584-g28deb56-qt-4.8.6-trusty-x86_64.tar.gz -O /tmp/vtk-v7.1.1-1584-g28deb56-qt-4.8.6-trusty-x86_64.tar.gz
  pushd /usr
  sudo tar -xvzf /tmp/vtk-v7.1.1-1584-g28deb56-qt-4.8.6-trusty-x86_64.tar.gz
  popd
}

if [ "$TRAVIS_OS_NAME" = "linux" ]; then
  sudo apt-get update -qq
  sudo apt-get install -y \
    build-essential \
    cmake \
    doxygen \
    graphviz \
    libexpat-dev \
    libfreetype6-dev \
    libglib2.0-dev \
    libjpeg-dev \
    libqt4-declarative \
    libqt4-dev \
    libqt4-private-dev \
    libtiff5-dev \
    libxml2-dev \
    libxt-dev \
    python-coverage \
    python-dev \
    python-lxml \
    python-numpy \
    python-sphinx \
    python-yaml \
    qt4-default \
    wget \
    xvfb

  sudo pip install --upgrade sphinx_rtd_theme breathe

  update_cmake_trusty
  fetch_vtk_trusty

  # start Xvfb for DISPLAY=:99.0
  /sbin/start-stop-daemon --start --quiet --pidfile /tmp/custom_xvfb_99.pid --make-pidfile \
                          --background --exec /usr/bin/Xvfb -- :99 -ac -screen 0 1280x1024x16

elif [ "$TRAVIS_OS_NAME" = "osx" ]; then

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
