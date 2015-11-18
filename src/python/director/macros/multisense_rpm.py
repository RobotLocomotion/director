secondsPerRevolution = QtGui.QInputDialog.getDouble(None, 'Set multisense revolution time', 'seconds per revolution', 4.0, 1.0, 60.0, 1)
perception.setMultisenseRevolutionTime(secondsPerRevolution)
