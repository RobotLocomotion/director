

set(DRAKE_DIR /source/drc/drc-trunk/software/drake)
set(DRAKE_LIB_DIR ${DRAKE_DIR}/pod-build/lib)

set(DRAKE_LIBRARIES
  ${DRAKE_LIB_DIR}/libdrakeRBM.dylib
  ${DRAKE_LIB_DIR}/libURDFRigidBodyManipulator.a
  ${DRAKE_LIB_DIR}/liburdf_interface.a
  )

set(DRAKE_INCLUDE_DIRS
  ${DRAKE_DIR}/systems/plants
  ${DRAKE_DIR}/systems/plants/tinyxml
  )


set(DRAKE_FOUND 1)
