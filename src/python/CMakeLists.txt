
set(python_files
  ddapp/affordance.py
  ddapp/applogic.py
  ddapp/cameracontrol.py
  ddapp/debugVis.py
  ddapp/ikeditor.py
  ddapp/ik.py
  ddapp/__init__.py
  ddapp/io.py
  ddapp/jointcontrol.py
  ddapp/lcmUtils.py
  ddapp/matlab.py
  ddapp/midi.py
  ddapp/objectmodel.py
  ddapp/perception.py
  ddapp/pointCloudUtils.py
  ddapp/segmentation.py
  ddapp/shallowCopy.py
  ddapp/simpletimer.py
  ddapp/spreadsheet.py
  ddapp/startup.py
  ddapp/tdx.py
  ddapp/timercallback.py
  ddapp/transformUtils.py
  ddapp/vtkAll.py
  ddapp/vtkNumpy.py

  #ddapp/macros/get_polydata.py
  ddapp/macros/publish.py
  ddapp/macros/bot_spy.py
  ddapp/macros/neck_pitch.py
  ddapp/macros/multisense_rpm.py

  ddapp/macros/segmentation_view/ortho_x.py
  ddapp/macros/segmentation_view/ortho_y.py
  ddapp/macros/segmentation_view/ortho_z.py
  ddapp/macros/segmentation_view/perspective.py
  ddapp/macros/segmentation_view/segment_board.py
  ddapp/macros/segmentation_view/segment_cinderblocks.py
  ddapp/macros/segmentation_view/segment_valve.py
  ddapp/macros/segmentation_view/major_planes.py

  )

set(python_module_dir "${LIBRARY_OUTPUT_PATH}/site-packages")

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


# Install python module
set(install_destination ${DD_INSTALL_LIB_DIR}/site-packages)

install(DIRECTORY  ${python_module_dir}/ddapp
        DESTINATION ${install_destination}
        COMPONENT Runtime
        USE_SOURCE_PERMISSIONS)


add_subdirectory(extensions)
