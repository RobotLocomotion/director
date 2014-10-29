
set(python_files
  ddapp/actionhandlers.py
  ddapp/actionmanagerpanel.py
  ddapp/affordance.py
  ddapp/affordancegraspupdater.py
  ddapp/affordanceitems.py
  ddapp/affordancelistener.py
  ddapp/applogic.py
  ddapp/asynctaskqueue.py
  ddapp/atlasdriver.py
  ddapp/atlasdriverpanel.py
  ddapp/botspy.py
  ddapp/callbacks.py
  ddapp/cameracontrol.py
  ddapp/cameraview.py
  ddapp/colorize.py
  ddapp/consoleapp.py
  ddapp/continuouswalkingdemo.py
  ddapp/debrisdemo.py
  ddapp/debugVis.py
  ddapp/depthimageprovider.py
  ddapp/drakevisualizer.py
  ddapp/drcargs.py
  ddapp/drilldemo.py
  ddapp/drilltaskpanel.py
  ddapp/fieldcontainer.py
  ddapp/filterUtils.py
  ddapp/footstepsdriver.py
  ddapp/footstepsdriverpanel.py
  ddapp/framevisualization.py
  ddapp/handcontrolpanel.py
  ddapp/handdriver.py
  ddapp/takktilevis.py
  ddapp/ik.py
  ddapp/ikconstraints.py
  ddapp/ikplanner.py
  ddapp/__init__.py
  ddapp/ioUtils.py
  ddapp/jointcontrol.py
  ddapp/lcmloggerwidget.py
  ddapp/lcmgl.py
  ddapp/lcmspy.py
  ddapp/lcmUtils.py
  ddapp/mapsregistrar.py
  ddapp/matlab.py
  ddapp/midi.py
  ddapp/multisensepanel.py
  ddapp/navigationpanel.py
  ddapp/objectmodel.py
  ddapp/perception.py
  ddapp/pfgraspdemo.py
  ddapp/planplayback.py
  ddapp/playbackpanel.py
  ddapp/pointpicker.py
  ddapp/propertyset.py
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
  ddapp/skybox.py
  ddapp/splinewidget.py
  ddapp/spreadsheet.py
  ddapp/startup.py
  ddapp/statelistener.py
  ddapp/tabledemo.py
  ddapp/tdx.py
  ddapp/teleoppanel.py
  ddapp/terrain.py
  ddapp/terrainitem.py
  ddapp/timercallback.py
  ddapp/transformUtils.py
  ddapp/trackers.py
  ddapp/utime.py
  ddapp/valvedemo.py
  ddapp/viewbehaviors.py
  ddapp/visualization.py
  ddapp/vtkAll.py
  ddapp/vtkNumpy.py

  ddapp/macros/popout.py
  ddapp/macros/publish.py

  ddapp/macros/segmentation_view/ortho_x.py
  ddapp/macros/segmentation_view/ortho_y.py
  ddapp/macros/segmentation_view/ortho_z.py
  ddapp/macros/segmentation_view/perspective.py

  actionmanager/__init__.py
  actionmanager/actions.py
  actionmanager/actionmanager.py
  actionmanager/simplefsm.py
  actionmanager/sequences.py

  ddapp/thirdparty/__init__.py
  ddapp/thirdparty/qhull_2d.py
  ddapp/thirdparty/min_bounding_rect.py
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



add_subdirectory(extensions)
add_subdirectory(tests)
