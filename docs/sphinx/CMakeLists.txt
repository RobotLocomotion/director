find_program(SPHINX_BUILD sphinx-build)
mark_as_advanced(SPHINX_BUILD)

if(NOT SPHINX_BUILD)
  message(STATUS "sphinx-build not found - python API documentation will not be built.")
  return()
endif()

set(doc_dir ${CMAKE_CURRENT_SOURCE_DIR})

add_custom_target(docs-sphinx-generate
  COMMAND ${CMAKE_COMMAND} -E remove_directory "${doc_dir}/generated"
  COMMAND sphinx-apidoc --separate -o "${doc_dir}/generated" "${DD_BINARY_DIR}/${DD_INSTALL_PYTHON_DIR}/director"
  )

add_custom_target(docs-sphinx-build
  COMMAND make -C "${doc_dir}" SPHINXBUILD=\"${EXECUTABLE_OUTPUT_PATH}/directorPython ${SPHINX_BUILD}\" html
  )
