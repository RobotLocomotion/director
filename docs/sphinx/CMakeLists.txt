find_program(SPHINX_BUILD sphinx-build)
mark_as_advanced(SPHINX_BUILD)

if(NOT SPHINX_BUILD)
  message(STATUS "sphinx-build not found - python API documentation will not be built.")
  return()
endif()

set(doc_dir ${CMAKE_CURRENT_BINARY_DIR}/generated)

add_custom_target(docs-sphinx
  COMMAND ${CMAKE_COMMAND} -E remove_directory "${doc_dir}"
  COMMAND sphinx-apidoc -F -o "${doc_dir}" "${DD_BINARY_DIR}/${DD_INSTALL_PYTHON_DIR}/ddapp"
  COMMAND make -C "${doc_dir}" SPHINXBUILD=\"${EXECUTABLE_OUTPUT_PATH}/ddConsoleApp ${SPHINX_BUILD}\" html
  )
