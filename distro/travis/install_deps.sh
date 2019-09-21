#!/bin/bash
set -ex

install_qt4_deps()
{
  apt-get install -y \
    libqt4-dev
}

install_qt5_deps()
{
  apt-get install -y \
    qtbase5-dev \
    qtbase5-private-dev \
    qttools5-dev \
    qtmultimedia5-dev \
    libqt5x11extras5-dev
}

install_ubuntu_deps_common()
{
  apt-get update
  apt-get install -y \
    build-essential \
    cmake \
    git \
    libglib2.0-dev \
    freeglut3-dev \
    libx11-dev \
    libxext-dev \
    libxt-dev \
    lsb-release \
    python3-coverage \
    python3-dev \
    python3-lxml \
    python3-numpy \
    python3-scipy \
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
    apt-get install -y libgl1-mesa-dev-lts-xenial
  else
    apt-get install -y libgl1-mesa-dev
  fi

  if [ "$ubuntu_version" = "18.04" ]; then
    install_qt5_deps
  else
    install_qt4_deps
  fi

  # cleanup to make the docker image smaller
  rm -rf /var/lib/apt/lists/*
}


install_osx_deps()
{
  brew update > brew_log.txt 2>&1 || cat brew_log.txt
  brew cleanup > brew_log.txt 2>&1 || cat brew_log.txt
  brew upgrade cmake qt python > brew_log.txt 2>&1 || cat brew_log.txt
  brew install vtk
  pip3 install coverage lxml PyYAML numpy scipy
}


if [ "$TRAVIS_OS_NAME" = "linux" ]; then
  install_ubuntu_deps
elif [ "$TRAVIS_OS_NAME" = "osx" ]; then
  install_osx_deps
fi
