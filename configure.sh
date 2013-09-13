
cmake=`which cmake`


$cmake \
  -DPYTHONQT_DIR=/source/PythonQt/install \
  -DQtPropertyBrowser_DIR=/source/qtpropertybrowser/build \
  -DCTK_PYTHONCONSOLE_DIR=/source/ctk/python_console/install \
  -DVTK_DIR=/source/paraview/build/VTK \
  -DPYTHON_INCLUDE_DIR:PATH=/usr/local/Frameworks/Python.framework/Headers \
  -DPYTHON_LIBRARY:FILEPATH=/usr/local/Frameworks/Python.framework/Versions/2.7/lib/libpython2.7.dylib \
   ../
