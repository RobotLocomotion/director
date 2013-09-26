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

  vtkDir=$HOME/source/paraview/build/VTK
  #vtkDir=/usr/lib/vtk-5.8

  #drakeDir=$HOME/source/drc/drc-trunk/software/drake
  drakeDir=$HOME/source/drake/drake
  
  $cmake \
    -DDRAKE_DIR=$drakeDir \
    -DPYTHONQT_DIR=$HOME/source/PythonQt/install \
    -DQtPropertyBrowser_DIR=$HOME/source/qt/build \
    -DCTK_PYTHONCONSOLE_DIR=$HOME/source/ctk/python_console/install \
    -DVTK_DIR=$vtkDir \
     ../
}

platform=`uname`
if [ $platform == 'Linux' ]; then

  configure_paladin09

elif [ $platform == 'Darwin' ]; then

  configure_trisol

fi


