#!/bin/bash



configure_trisol()
{
  cmake=`which cmake`

  $cmake \
    -DDRAKE_DIR=/source/drc/drc-trunk/software/drake \
    -DPYTHONQT_DIR=/source/PythonQt/install \
    -DQtPropertyBrowser_DIR=/source/qtpropertybrowser/build \
    -DCTK_PYTHONCONSOLE_DIR=/source/ctk/python_console/install \
    -DVTK_DIR=/source/paraview/build/VTK \
    -DPYTHON_INCLUDE_DIR:PATH=/usr/local/Frameworks/Python.framework/Headers \
    -DPYTHON_LIBRARY:FILEPATH=/usr/local/Frameworks/Python.framework/Versions/2.7/lib/libpython2.7.dylib \
     ../
}

configure_paladin09()
{
  cmake=`which cmake`

  $cmake \
    -DDRAKE_DIR=/home/pat/source/drc/drc-trunk/software/drake \
    -DPYTHONQT_DIR=/home/pat/source/PythonQt/install \
    -DQtPropertyBrowser_DIR=/home/pat/source/qt/build \
    -DCTK_PYTHONCONSOLE_DIR=/home/pat/source/ctk/python_console/install \
    -DVTK_DIR=/home/pat/source/paraview/build/VTK \
     ../
}

platform=`uname`
if [ $platform == 'Linux' ]; then

  configure_paladin09

elif [ $platform == 'Darwin' ]; then

  configure_trisol

fi


