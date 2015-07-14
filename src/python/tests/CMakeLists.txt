if(NOT BUILD_TESTING)
  return()
endif()

include(python-coverage/setup.cmake)

set(python_tests_core
  testConsoleApp.py
  testFrameSync.py
  testObjectModel.py
  testPropertiesPanel.py
  testPythonConsole.py
  testTaskQueue.py
  testTransformations.py
)

set(python_tests_robot
  testActionSequence.py
  testAffordanceItems.py
  testAtlasDriver.py
  testCameraView.py
  testContinuousWalking.py
  testDrakeVisualizer.py
  testImageView.py
  testKinematicPose.py
  testLoadUrdf.py
  testOtdfParser.py
  testPlanConstraints.py
  testRobotSystem.py
  testTableFit.py
  testTableFitStereo.py
  testTeleopPanel.py
  testValveFit.py
  testValveFitStereo.py
  testAmazonPod.py
)

set(testKinematicPose_extra_args -v5)

set(python_test_args
  --testing --data-dir ${CMAKE_SOURCE_DIR}/../../../drc-testing-data --output-dir ${CMAKE_BINARY_DIR}/Testing/Temporary
)

set(python_exe ${CMAKE_BINARY_DIR}/bin/ddConsoleApp)

macro(add_python_test name label)
  get_filename_component(base_name ${name} NAME_WE)
  set(test_name test_${base_name})
  add_test(${test_name} ${python_exe} ${CMAKE_CURRENT_SOURCE_DIR}/${name} ${python_test_args} ${${base_name}_extra_args})
  set_property(TEST ${test_name} PROPERTY ENVIRONMENT ${python_coverage_environment_arg})
  set_property(TEST ${test_name} PROPERTY LABELS ${label})
endmacro()


foreach(name ${python_tests_core})
  add_python_test(${name} core)
endforeach()

foreach(name ${python_tests_robot})
  add_python_test(${name} robot)
endforeach()
