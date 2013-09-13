macro(setup_qt4)
  find_package(Qt4 REQUIRED QtCore QtGui QtScript)
  include(${QT_USE_FILE})
endmacro()
