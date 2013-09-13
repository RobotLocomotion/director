

set(DRAKE_DIR /source/drc/drc-trunk/software/drake)


set(DRAKE_LIBRARIES
  ${DRAKE_DIR}/pod-build/lib/libdrakeRBM.dylib
  ${DRAKE_DIR}/pod-build/lib/libURDFRigidBodyManipulator.a
  )

set(DRAKE_INCLUDE_DIRS
  ${DRAKE_DIR}/systems/plants
  )


set(DRAKE_FOUND 1)
