
set(python_files
  director/__init__.py
  director/actionhandlers.py
  director/affordanceupdater.py
  director/affordanceitems.py
  director/affordancemanager.py
  director/affordancepanel.py
  director/affordanceurdf.py
  director/applogic.py
  director/appsettings.py
  director/assimp2vtk.py
  director/asynctaskqueue.py
  director/atlasdriver.py
  director/atlasdriverpanel.py
  director/bihandeddemo.py
  director/blackoutmonitor.py
  director/botspy.py
  director/callbacks.py
  director/camerabookmarks.py
  director/cameracontrol.py
  director/cameracontrolpanel.py
  director/cameraview.py
  director/colorize.py
  director/componentgraph.py
  director/consoleapp.py
  director/continuouswalkingdemo.py
  director/copmonitor.py
  director/coursemodel.py
  director/debrisdemo.py
  director/debugVis.py
  director/depthimageprovider.py
  director/depthscanner.py
  director/doordemo.py
  director/drakevisualizer.py
  director/drcargs.py
  director/drilldemo.py
  director/drivingplanner.py
  director/egressplanner.py
  director/frameupdater.py
  director/fieldcontainer.py
  director/filterUtils.py
  director/footstepsdriver.py
  director/footstepsdriverpanel.py
  director/framevisualization.py
  director/gamepad.py
  director/geometryencoder.py
  director/handcontrolpanel.py
  director/handdriver.py
  director/takktilevis.py
  director/ikconstraintencoder.py
  director/ikconstraints.py
  director/ikparameters.py
  director/ikplanner.py
  director/imageview.py
  director/ioUtils.py
  director/irisUtils.py
  director/irisdriver.py
  director/jointcontrol.py
  director/jointpropagator.py
  director/korgnano.py
  director/lcmframe.py
  director/lcmloggerwidget.py
  director/lcmgl.py
  director/lcmobjectcollection.py
  director/lcmoctomap.py
  director/lcmcollections.py  
  director/lcmspy.py
  director/lcmUtils.py
  director/mainwindowapp.py
  director/mapsregistrar.py
  director/matlab.py
  director/matlabik.py
  director/meshmanager.py
  director/midi.py
  director/multisensepanel.py
  director/navigationpanel.py
  director/objectmodel.py
  director/otdfmodel.py
  director/openscope.py
  director/packagepath.py
  director/perception.py
  director/planningutils.py
  director/planplayback.py
  director/playbackpanel.py
  director/pointcloudlcm.py
  director/pointpicker.py
  director/polarisplatformplanner.py
  director/propertyanimation.py
  director/propertyset.py
  director/pydrakeik.py
  director/raycastdriver.py
  director/robotlinkselector.py
  director/robotstate.py
  director/robotplanlistener.py
  director/robotposegui.py
  director/robotsystem.py
  director/roboturdf.py
  director/robotviewbehaviors.py
  director/screengrabberpanel.py
  director/segmentation.py
  director/segmentationpanel.py
  director/segmentationroutines.py
  director/sensordatarequestpanel.py
  director/shallowCopy.py
  director/simpletimer.py
  director/sitstandplanner.py
  director/skybox.py
  director/splinewidget.py
  director/spreadsheet.py
  director/startup.py
  director/statelistener.py
  director/surprisetask.py
  director/switchplanner.py
  director/tasklaunchpanel.py
  director/tdx.py
  director/teleoppanel.py
  director/motionplanningpanel.py
  director/terrain.py
  director/terrainitem.py
  director/terraintask.py
  director/timercallback.py
  director/transformUtils.py
  director/trackers.py
  director/utime.py
  director/uuidutil.py
  director/valvedemo.py
  director/version.py.in
  director/viconvisualizer.py
  director/viewbehaviors.py
  director/viewcolors.py
  director/vieweventfilter.py
  director/visualization.py
  director/vtkAll.py
  director/vtkNumpy.py
  director/walkingtestdemo.py
  director/wristforcetorquevisualizer.py
  director/kinectlcm.py
  director/plannerPublisher.py


  director/macros/segmentation_view/ortho_x.py
  director/macros/segmentation_view/ortho_y.py
  director/macros/segmentation_view/ortho_z.py
  director/macros/segmentation_view/perspective.py

  director/tasks/__init__.py
  director/tasks/robottasks.py
  director/tasks/taskmanagerwidget.py
  director/tasks/taskuserpanel.py
  director/tasks/descriptions/__init__.py
  director/tasks/descriptions/drillTask.py
  director/tasks/descriptions/taskLibrary.py

  director/thirdparty/__init__.py
  director/thirdparty/qhull_2d.py
  director/thirdparty/min_bounding_rect.py
  director/thirdparty/NumericStringParser.py
  director/thirdparty/numpyjsoncoder.py
  director/thirdparty/transformations.py
  director/thirdparty/conversions.py
  director/thirdparty/geometry_msgs_mock.py
  director/thirdparty/naming.py
  director/thirdparty/pysdf.py
  director/thirdparty/toposort.py

  director/sceneloader.py

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
  get_filename_component(tgtDir ${tgt} PATH)
  get_filename_component(tgtExt ${tgt} EXT)
  get_filename_component(tgt_we ${tgt} NAME_WE)

  file(MAKE_DIRECTORY ${tgtDir})

  if(${tgtExt} STREQUAL ".py.in")
    set(tgt "${tgtDir}/${tgt_we}.py")
    configure_file(${src} ${tgt} @ONLY)
  else()
    add_custom_command(DEPENDS ${src}
                       COMMAND ${CMAKE_COMMAND} -E copy ${src} ${tgt}
                       OUTPUT ${tgt}
                       COMMENT "source copy")
  endif()


  set(copied_python_files ${copied_python_files} ${tgt})
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
  director/macros/popout.py
  director/macros/publish.py
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
