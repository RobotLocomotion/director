
set(python_files
  ddapp/actionhandlers.py
  ddapp/affordanceupdater.py
  ddapp/affordanceitems.py
  ddapp/affordancemanager.py
  ddapp/affordancepanel.py
  ddapp/affordanceurdf.py
  ddapp/applogic.py
  ddapp/asynctaskqueue.py
  ddapp/atlasdriver.py
  ddapp/atlasdriverpanel.py
  ddapp/bihandeddemo.py
  ddapp/blackoutmonitor.py
  ddapp/botspy.py
  ddapp/callbacks.py
  ddapp/camerabookmarks.py
  ddapp/cameracontrol.py
  ddapp/cameraview.py
  ddapp/colorize.py
  ddapp/consoleapp.py
  ddapp/continuouswalkingdemo.py
  ddapp/copmonitor.py
  ddapp/coursemodel.py
  ddapp/debrisdemo.py
  ddapp/debugVis.py
  ddapp/depthimageprovider.py
  ddapp/doordemo.py
  ddapp/drakevisualizer.py
  ddapp/drcargs.py
  ddapp/drilldemo.py
  ddapp/drilltaskpanel.py
  ddapp/drivingplanner.py
  ddapp/egressplanner.py
  ddapp/frameupdater.py
  ddapp/fieldcontainer.py
  ddapp/filterUtils.py
  ddapp/footstepsdriver.py
  ddapp/footstepsdriverpanel.py
  ddapp/framevisualization.py
  ddapp/gamepad.py
  ddapp/geometryencoder.py
  ddapp/handcontrolpanel.py
  ddapp/handdriver.py
  ddapp/takktilevis.py
  ddapp/ik.py
  ddapp/ikconstraints.py
  ddapp/ikparameters.py
  ddapp/ikplanner.py
  ddapp/ikconstraintencoder.py
  ddapp/__init__.py
  ddapp/ioUtils.py
  ddapp/irisUtils.py
  ddapp/irisdriver.py
  ddapp/jointcontrol.py
  ddapp/jointpropagator.py
  ddapp/kinematicposeplanner.py
  ddapp/lcmloggerwidget.py
  ddapp/lcmgl.py
  ddapp/lcmobjectcollection.py
  ddapp/lcmspy.py
  ddapp/lcmUtils.py
  ddapp/mappingdemo.py
  ddapp/mappingpanel.py
  ddapp/mapsregistrar.py
  ddapp/matlab.py
  ddapp/meshmanager.py
  ddapp/midi.py
  ddapp/multisensepanel.py
  ddapp/navigationpanel.py
  ddapp/objectmodel.py
  ddapp/otdfmodel.py
  ddapp/openscope.py
  ddapp/perception.py
  ddapp/pfgrasp.py
  ddapp/pfgrasppanel.py
  ddapp/planplayback.py
  ddapp/playbackpanel.py
  ddapp/pointpicker.py
  ddapp/polarisplatformplanner.py
  ddapp/propertyset.py
  ddapp/raycastdriver.py
  ddapp/robotstate.py
  ddapp/robotplanlistener.py
  ddapp/robotsystem.py
  ddapp/roboturdf.py
  ddapp/screengrabberpanel.py
  ddapp/segmentation.py
  ddapp/segmentationpanel.py
  ddapp/segmentationroutines.py
  ddapp/sensordatarequestpanel.py
  ddapp/shallowCopy.py
  ddapp/simpletimer.py
  ddapp/sitstandplanner.py
  ddapp/skybox.py
  ddapp/splinewidget.py
  ddapp/spreadsheet.py
  ddapp/startup.py
  ddapp/statelistener.py
  ddapp/surprisetask.py
  ddapp/switchplanner.py
  ddapp/tabledemo.py
  ddapp/tasklaunchpanel.py
  ddapp/tdx.py
  ddapp/teleoppanel.py
  ddapp/terrain.py
  ddapp/terrainitem.py
  ddapp/terraintask.py
  ddapp/timercallback.py
  ddapp/transformUtils.py
  ddapp/trackers.py
  ddapp/utime.py
  ddapp/uuidutil.py
  ddapp/valvedemo.py
  ddapp/viewbehaviors.py
  ddapp/visualization.py
  ddapp/vtkAll.py
  ddapp/vtkNumpy.py
  ddapp/walkingtestdemo.py
  ddapp/wristforcetorquevisualizer.py
  ddapp/kinectlcm.py
  ddapp/plannerPublisher.py


  ddapp/macros/segmentation_view/ortho_x.py
  ddapp/macros/segmentation_view/ortho_y.py
  ddapp/macros/segmentation_view/ortho_z.py
  ddapp/macros/segmentation_view/perspective.py

  ddapp/tasks/__init__.py
  ddapp/tasks/robottasks.py
  ddapp/tasks/taskmanagerwidget.py
  ddapp/tasks/taskuserpanel.py
  ddapp/tasks/descriptions/__init__.py
  ddapp/tasks/descriptions/drillTask.py
  ddapp/tasks/descriptions/taskLibrary.py

  ddapp/thirdparty/__init__.py
  ddapp/thirdparty/qhull_2d.py
  ddapp/thirdparty/min_bounding_rect.py
  ddapp/thirdparty/NumericStringParser.py
  ddapp/thirdparty/numpyjsoncoder.py
  ddapp/thirdparty/transformations.py

  urdf_parser_py/__init__.py
  urdf_parser_py/sdf.py
  urdf_parser_py/urdf.py
  urdf_parser_py/xml_reflection/__init__.py
  urdf_parser_py/xml_reflection/basics.py
  urdf_parser_py/xml_reflection/core.py
  )


set(python_module_dir "${CMAKE_BINARY_DIR}/${DD_INSTALL_PYTHON_DIR}")

# Copy python files
set(copied_python_files)
foreach(file ${python_files})
  set(src "${CMAKE_CURRENT_SOURCE_DIR}/${file}")
  set(tgt "${python_module_dir}/${file}")
  set(copied_python_files ${copied_python_files} ${tgt})
  get_filename_component(tgtDir ${tgt} PATH)
  file(MAKE_DIRECTORY ${tgtDir})
  add_custom_command(DEPENDS ${src}
                     COMMAND ${CMAKE_COMMAND} -E copy ${src} ${tgt}
                     OUTPUT ${tgt}
                     COMMENT "source copy")

  get_filename_component(tgt_we ${tgt} NAME_WE)
  get_filename_component(installDirSuffix ${file} PATH)
  install(FILES ${tgt} ${tgtDir}/${tgt_we}.pyc DESTINATION ${DD_INSTALL_PYTHON_DIR}/${installDirSuffix} COMPONENT Runtime)
endforeach(file)


# Byte compile python files
set(compile_all_script "${CMAKE_CURRENT_BINARY_DIR}/compile_all.py")
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/compile_all.py.in"
               ${compile_all_script}
               @ONLY IMMEDIATE)

add_custom_command(
  COMMAND ${PYTHON_EXECUTABLE}
  ARGS  "${compile_all_script}"
  DEPENDS ${copied_python_files}  ${compile_all_script}
  OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/compile_complete"
  )

add_custom_target(python_byte_compile ALL DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/compile_complete")

# cleanup old python files if needed
set(cleanup_files
  ddapp/macros/popout.py
  ddapp/macros/publish.py
)
foreach(file ${cleanup_files})
  foreach(search_dir "${python_module_dir}" "${CMAKE_INSTALL_PREFIX}/${DD_INSTALL_PYTHON_DIR}")
    if (EXISTS "${search_dir}/${file}")
      file(REMOVE "${search_dir}/${file}")
    endif()
    if (EXISTS "${search_dir}/${file}c")
      file(REMOVE "${search_dir}/${file}c")
    endif()
  endforeach()
endforeach()


add_subdirectory(extensions)
add_subdirectory(tests)
