configure_file(ddVersion.h.in ddVersion.h)


install_headers(
  ${CMAKE_CURRENT_BINARY_DIR}/ddVersion.h
  ddMacros.h
  ddOpenGL.h
  ddSharedPtr.h
  )
