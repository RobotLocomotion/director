#!/bin/bash


case $1 in
  ("homebrew")
    brew tap homebrew/science
    brew tap homebrew/python
    brew install python numpy
    brew install vtk5 --with-qt ;;
  ("macports")
    port install qt4-mac qwt
    port install python27 py-pip py-numpy
    port install vtk5 +qt4_mac +python27 ;;
  ("ubuntu")
    apt-get install libqt4-dev libvtk5-dev libvtk5-qt4-dev python-vtk python-numpy ;;
  ("ubuntu_16_04")
    apt install \
      build-essential \
      cmake \
      debhelper \
      doxygen \
      f2c \
      freeglut3-dev \
      gfortran \
      graphviz \
      gtk-doc-tools \
      libblas-dev \
      libboost-filesystem-dev \
      libboost-iostreams-dev \
      libboost-program-options-dev \
      libboost-random-dev \
      libboost-regex-dev \
      libboost-signals-dev \
      libboost-system-dev \
      libboost-thread-dev \
      libf2c2-dev \
      libfreeimage-dev \
      libglew-dev \
      libglib2.0-dev \
      libgmp3-dev \
      libgsl0-dev \
      libgtkmm-2.4-dev \
      liblapack-dev \
      libltdl-dev \
      libopenni-dev \
      libportmidi-dev \
      libprotobuf-dev \
      libprotoc-dev \
      libqglviewer-dev \
      libqhull-dev \
      libqt4-dev \
      libqwt-dev \
      libsdl1.2-dev \
      libspnav-dev \
      libsuitesparse-dev \
      libtar-dev \
      libtbb-dev \
      libterm-readkey-perl \
      libtinyxml-dev \
      libusb-1.0-0-dev \
      libv4l-dev \
      libvtk5-dev \
      libvtk5-qt4-dev \
      libwww-perl \
      libx264-dev \
      libxml2-dev \
      libxmu-dev \
      mercurial \
      ncurses-dev \
      pkg-config \
      protobuf-compiler \
      python-dev \
      python-matplotlib \
      python-numpy \
      python-pip \
      python-pygame \
      python-pymodbus \
      python-scipy \
      python-vtk \
      python-yaml \
      spacenavd \
      subversion \
      swig
  ("cygwin")
    echo "WARNING: install_prereqs cygwin not implemented for this module" ;;
  (*)
    echo "Usage: ./install_prereqs.sh package_manager"
    echo "where package_manager is one of the following: "
    echo "  homebrew"
    echo "  macports"
    echo "  ubuntu"
    echo "  ubuntu_16_04"
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
