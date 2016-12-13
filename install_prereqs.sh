#!/bin/bash

case $1 in
  ("homebrew")
    brew tap homebrew/science && brew tap homebrew/python && brew install cmake && brew install python numpy && brew install vtk5 --with-qt ;;
  ("ubuntu")
    apt-get install cmake libvtk5-qt4-dev python-dev python-vtk python-numpy ;;
  (*)
    echo "Usage: ./install_prereqs.sh package_manager"
    echo "where package_manager is one of the following: "
    echo "  homebrew"
    echo "  ubuntu"
    exit 1 ;;
esac