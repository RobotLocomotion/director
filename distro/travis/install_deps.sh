#!/bin/bash
set -ex

install_ubuntu_deps_common()
{
  apt-get update
  apt-get install -y \
    build-essential \
    cmake \
    git \
    libexpat1-dev \
    libfreetype6-dev \
    libglib2.0-dev \
    libhdf5-dev \
    libjpeg8-dev \
    libjsoncpp-dev \
    liblz4-dev \
    libnetcdf-dev \
    libogg-dev \
    libpng12-dev \
    libqt4-dev \
    libtheora-dev \
    libtiff5-dev \
    libx11-dev \
    libxext-dev \
    libxml2-dev \
    libxt-dev \
    lsb-release \
    python-coverage \
    python-dev \
    python-lxml \
    python-numpy \
    python-scipy \
    python-yaml \
    wget \
    xvfb \
    zlib1g-dev

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
    apt-get install -y libgl1-mesa-dev libnetcdf-cxx-legacy-dev
  fi

  # cleanup to make the docker image smaller
  rm -rf /var/lib/apt/lists/*
}

install_osx_deps()
{
  brew tap robotlocomotion/director
  brew tap-pin robotlocomotion/director

  brew update > brew_update_log.txt

  # checkout a fixed version of homebrew/core formula from September 1st 2017
  export HOMEBREW_NO_AUTO_UPDATE=1
  cd $(brew --repository homebrew/core) && git reset --hard 31da4bf0c8ffeb6b2b7badd796676a91129d27d5
  #brew upgrade

  brew install libyaml vtk@8.0
  brew install doxygen graphviz
  brew install glib # for lcm
  brew ls --versions python || brew install python
  brew ls --versions numpy || brew install numpy || echo "error on brew install numpy"

  pip2 install coverage lxml PyYAML Sphinx sphinx_rtd_theme
}



if [ "$TRAVIS_OS_NAME" = "linux" ]; then
  install_ubuntu_deps
elif [ "$TRAVIS_OS_NAME" = "osx" ]; then
  install_osx_deps
fi
