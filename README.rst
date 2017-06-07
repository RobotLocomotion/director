========
Director
========

.. contents:: Table of Contents

Introduction
============

This README describes how to download and build the Director source code
and how to satisfy 3rd party dependencies.


Background
----------

The Director is a robotics interface and visualization framework.

It includes applications for working with `Drake <http://drake.mit.edu>`_,
and includes the primary user interface used by Team MIT in the DARPA Robotics Challenge.

`Team MIT DRC day-1 visualization <https://www.youtube.com/watch?v=em69XtIEEAg>`_

The Director is a collection of C++ and Python libraries and applications.  Many components from
this repository are usable out-of-the-box, but some require additional components from
the greater MIT DRC codebase.

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

    brew tap homebrew/python
    brew tap patmarion/director
    brew install cmake python numpy vtk5 eigen

The dependencies can be installed on Ubuntu using apt-get:

::

    sudo apt-get install cmake libvtk5-qt4-dev libeigen3-dev python-dev python-vtk python-numpy

If you are running Ubuntu 16.04, please install the following non-conservative set of
dependencies by executing:

::

    ./distro/pods/drake-distro/install_prereqs.sh ubuntu_16_04

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


Documentation
=============

A preliminary Online Help for the Director (currently in preparation) can be found `here <https://openhumanoids.github.io/director/>`_.


Citing
======

If you wish to cite the director, please use:

::

    @misc{director,
      author = "Pat Marion",
      title = "Director: A robotics interface and visualization framework",
      year = 2015,
      url = "http://github.com/RobotLocomotion/director"
    }
