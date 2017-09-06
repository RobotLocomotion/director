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
    python3-coverage \
    python3-dev \
    python3-lxml \
    python3-numpy \
    python3-yaml \
    wget \
    xvfb

  if [ "$MAKE_DOCS" = "ON" ]; then
    apt-get install -y \
      doxygen \
      graphviz \
      python3-pip \
      python3-sphinx
    pip3 install sphinx-rtd-theme
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
  brew tap patmarion/director
  brew tap-pin patmarion/director

  brew update > brew_update_log.txt

  # checkout a fixed version of homebrew/core formula from August 12th 2017
  export HOMEBREW_NO_AUTO_UPDATE=1
  cd $(brew --repository homebrew/core) && git reset --hard 543d8e9b27b9bb25ce4491773684c30c8cb66dcc
  #brew upgrade

  brew install qt vtk7python3
  brew install doxygen graphviz
  brew install glib # for lcm
  brew ls --versions python3 || brew install python3
  #brew ls --versions numpy || brew install numpy --with-python3 || echo "error on brew install numpy"

  pip3 install numpy pyyaml lxml Sphinx sphinx-rtd-theme coverage
}



if [ "$TRAVIS_OS_NAME" = "linux" ]; then
  install_ubuntu_deps
elif [ "$TRAVIS_OS_NAME" = "osx" ]; then
  install_osx_deps
fi
