==============================
Compiling Director from source
==============================

System Requirements
-------------------

As of this writing, the software is supported on Ubuntu 14.04, MacOSX 10.10,
and Microsoft Windows 8 using the system native compilers.  It likely also
works on similar platforms and OS versions.


Download Instructions
=====================

Install Git
-----------

The source code is stored in a Git repository. To download the
source code you may need to first install Git on your system.
On Mac, we recommend using Homebrew.  On Windows, download the
official git package from https://git-scm.com


Download the source code
------------------------

Download the repository with the ``git clone`` command:

::

    git clone https://github.com/RobotLocomotion/director.git


Dependencies
============


Required Dependencies
-----------------

The required 3rd party dependencies are:

  - Qt 4.8
  - VTK 5.8 or 5.10
  - Python 2.7 and numpy

Additionally, you will need CMake 2.8 or greater to configure the source code.

The dependencies can be installed on Mac using `Homebrew <http://brew.sh/>`_:

::

    brew tap homebrew/science
    brew tap homebrew/python
    brew install cmake
    brew install python numpy
    brew install vtk5 --with-qt

The dependencies can be installed on Ubuntu using apt-get:

::

    sudo apt-get install cmake libvtk5-qt4-dev python-dev python-vtk python-numpy




Building
========


Compiling
---------

::

    make superbuild

This is an alias for:

::

    mkdir build
    cd build
    cmake ../distro/superbuild
