Running tests
=============


CTest
-----

CTest is a testing framework provided by CMake.  Tests are described in
a CMakeLists.txt file that is part of the Director build system.  For example,
many tests are declared in the file ``src/python/testing/CMakeLists.txt``.


Dashboard
---------

A build machine automatically builds and runs tests.  The testing results are
submitted to a server and displayed on the `project dashboard <http://kobol.csail.mit.edu/cd/index.php?project=DRC>`_.


Running tests
-------------

You can run tests from the build directory using the ctest command.
First locate the build directory.  Depending on which project you are working
with, the Director build directory is placed in a different location.

=============  =======================
Build type     Build location
=============  =======================
openhumanoids  software/director/distro/pods/drc/pod-build
drake          externals/director/distro/pods/drake-distro/pod-build/src/director-build
superbuild     <build-root>/src/director-build
=============  =======================

Next, cd to the build directory and then run this command to list the tests::

    ctest -N

The -N flag means list tests but don't run them.  If you don't see any tests
listed, then the build was configured with ``BUILD_TESTING=OFF``.  Reconfigure
and set the BUILD_TESTING option to ON.

To run all tests, just run ctest without arguments::

    ctest

The ctest command can take optional flags on the command line to control its behavior.
To run tests with verbose information displayed, use the flags ``-V`` or ``-VV``.  This is
helpful to see the specific command line that is used to run an individual test.
You can copy and paste that command line to run the test directly instead of running
the test through the ctest launcher.  This can be useful if you want to run a test via gdb
or valgrind, for example.

You can specify a regex or string match to run a subset of tests with matching
names.  For example, ``ctest -R robot`` will run all tests with the string robot
in their name.


Testing data
------------

Some tests require extra data files to run.  The data files are stored in
an external repository because of larger file sizes.  The testing data
is `located on Github <https://github.com/openhumanoids/drc-testing-data>`_.
Clone the git repository::

    git clone https://github.com/openhumanoids/drc-testing-data

The testing data repository is designed for running tests from the ``openhumanoids/main-distro``
project.  Place the testing data directory in the same folder that contains the
main-distro folder.
