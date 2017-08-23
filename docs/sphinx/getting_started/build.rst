==============================
Compiling Director from source
==============================

System Requirements
-------------------

As of this writing, the software is tested on Ubuntu 14.04 and 16.04, and MacOSX 10.11.
The build should work on Microsoft Windows with MSVC but it is not continuously tested.
In theory it can run on any platform where VTK and Qt are supported.


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
---------------------

The required 3rd party dependencies are:

  - Qt4 or Qt5 (Qt 4.8.7 recommended)
  - VTK 6.2+ (VTK 7.1.1 recommended)
  - Python 2.7 and NumPy

Additionally, you will need CMake 2.8 or greater to configure the source code.

The dependencies can be installed on Mac using `Homebrew <http://brew.sh/>`_:

::

    brew tap homebrew/python
    brew tap patmarion/director && brew tap-pin patmarion/director
    brew install cmake python numpy qt vtk7 eigen

The dependencies can be installed on Ubuntu using apt-get:

::

    sudo apt-get install cmake libqt4-dev python-dev python-numpy

On Ubuntu the build does not require VTK to be installed.  A compatible version
of VTK will be downloaded (precompiled binaries) at build time.


Building
========

Compiling
---------

::

    make superbuild

This is an alias for:

::

    mkdir build && cd build
    cmake ../distro/superbuild
    make
