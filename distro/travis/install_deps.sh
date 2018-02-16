#!/bin/bash
set -ex

install_ubuntu_deps_common()
{
  apt-get update
  apt-get install -y \
    build-essential \
    cmake \
    git \
    libglib2.0-dev \
    libqt4-dev \
    libx11-dev \
    libxext-dev \
    libxt-dev \
    lsb-release \
    python-coverage \
    python-dev \
    python-lxml \
    python-numpy \
    python-scipy \
    python-yaml \
    python-zmq \
    wget \
    xvfb

  if [ "$MAKE_DOCS" = "ON" ]; then
    apt-get install -y \
      doxygen \
      graphviz \
      python-pip \
      python-sphinx
    pip install sphinx_rtd_theme
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
    apt-get install -y libgl1-mesa-dev-lts-xenial
  else
    apt-get install -y libgl1-mesa-dev
  fi

  # cleanup to make the docker image smaller
  rm -rf /var/lib/apt/lists/*
}

install_osx_deps()
{
  export HOMEBREW_NO_AUTO_UPDATE=1

  brew tap patmarion/director
  brew tap-pin patmarion/director

  # don't update for now because we will used a fixed version
  #brew update > brew_update_log.txt

  # checkout a fixed version of homebrew/core formula from August 12th 2017
  cd $(brew --repository homebrew/core) && git fetch && git reset --hard 543d8e9b27b9bb25ce4491773684c30c8cb66dcc

  brew install doxygen graphviz libyaml vtk7 || echo "error on brew install"
  brew install glib # for lcm
  brew ls --versions python || brew install python
  brew ls --versions numpy || brew install numpy || echo "error on brew install numpy"
  brew install zeromq

  pip install coverage lxml PyYAML Sphinx sphinx_rtd_theme pyzmq
}



if [ "$TRAVIS_OS_NAME" = "linux" ]; then
  install_ubuntu_deps
elif [ "$TRAVIS_OS_NAME" = "osx" ]; then
  install_osx_deps
fi
