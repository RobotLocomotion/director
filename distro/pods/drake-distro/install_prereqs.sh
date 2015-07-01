#!/bin/bash


case $1 in
  ("homebrew")
    brew tap homebrew/science
    brew tap homebrew/python
    brew install python numpy
    brew install vtk5 --with-qt ;;
  ("macports")
    echo "WARNING: install_prereqs macports not implemented for this module" ;;
  ("ubuntu")
    apt-get install libqt4-dev libvtk5-dev libvtk5-qt4-dev python-vtk python-numpy ;;
  ("cygwin")
    echo "WARNING: install_prereqs cygwin not implemented for this module" ;;
  (*)
    echo "Usage: ./install_prereqs.sh package_manager"
    echo "where package_manager is one of the following: "
    echo "  homebrew"
    echo "  macports"
    echo "  ubuntu"
    echo "  cygwin"
    exit 1 ;;
esac

if [ -f tobuild.txt ]; then
  SUBDIRS=`grep -v "^\#" tobuild.txt`
  for subdir in $SUBDIRS; do
    if [ -f $subdir/install_prereqs.sh ]; then
      echo "installing prereqs for $subdir"
      ( cd $subdir; ./install_prereqs.sh $1 || true )
    fi
  done
fi
