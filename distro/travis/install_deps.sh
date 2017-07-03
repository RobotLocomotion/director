#!/bin/bash
set -ex

install_ubuntu_deps_common()
{
  apt-get update
  apt-get install -y \
    build-essential \
    cmake \
    freeglut3-dev \
    git \
    libglib2.0-dev \
    libqt4-dev \
    lsb-release \
    python-coverage \
    python-dev \
    python-lxml \
    python-numpy \
    python-yaml \
    wget \
    xvfb

  if [ "$MAKE_DOCS" = "ON" ]; then
    apt-get install -y \
      doxygen \
      graphviz \
      python-pip \
      python-sphinx
    pip install sphinx-rtd-theme
  fi

  if [ "$MAKE_PACKAGE" = "ON" ]; then
    apt-get install -y \
      curl
  fi

}

install_ubuntu_deps()
{
  install_ubuntu_deps_common

  ubuntu_version=$(lsb_release --release --short)
  if [ "$ubuntu_version" = "14.04" ]; then
   :
  else
   :
  fi

  # cleanup to make the docker image smaller
  rm -rf /var/lib/apt/lists/*
}

install_osx_deps()
{
  brew tap homebrew/python
  brew tap homebrew/science
  brew tap patmarion/director
  brew tap-pin patmarion/director

  brew update > brew_update_log.txt
  #brew upgrade

  brew install qt vtk7
  brew install doxygen graphviz
  brew install glib # for lcm
  brew ls --versions python || brew install python
  brew ls --versions numpy || brew install numpy || echo "error on brew install numpy"

  pip install pyyaml lxml Sphinx sphinx-rtd-theme coverage
}



if [ "$TRAVIS_OS_NAME" = "linux" ]; then
  install_ubuntu_deps
elif [ "$TRAVIS_OS_NAME" = "osx" ]; then
  install_osx_deps
fi
