if(NOT BUILD_TESTING)
  return()
endif()

include(python-coverage/setup.cmake)

set(python_tests
  testActionSequence.py
  testAffordanceItems.py
  testAtlasDriver.py
  testCameraView.py
  testConsoleApp.py
  testContinuousWalking.py
  testDrakeVisualizer.py
  testFrameSync.py
  testImageView.py
  testObjectModel.py
  testOtdfParser.py
  testPropertiesPanel.py
  testPythonConsole.py
  testRobotSystem.py
  testTableFit.py
  testTableFitStereo.py
  testTaskQueue.py
  testValveFit.py
  testValveFitStereo.py
  testAmazonPod.py
)

set(python_test_args
  --testing --data-dir ${CMAKE_SOURCE_DIR}/../../../drc-testing-data --output-dir ${CMAKE_BINARY_DIR}/Testing/Temporary
)

set(python_exe ${CMAKE_BINARY_DIR}/bin/ddConsoleApp)

macro(add_python_test name)
  get_filename_component(base_name ${name} NAME_WE)
  set(test_name test_${base_name})
  add_test(${test_name} ${python_exe} ${CMAKE_CURRENT_SOURCE_DIR}/${name} ${python_test_args} ${${base_name}_extra_args})
  set_property(TEST ${test_name} PROPERTY ENVIRONMENT ${python_coverage_environment_arg})
endmacro()


foreach(name ${python_tests})
  add_python_test(${name})
endforeach()
