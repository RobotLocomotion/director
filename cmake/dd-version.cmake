SET(DD_VERSION_MAJOR 0)
SET(DD_VERSION_MINOR 1)
SET(DD_VERSION_PATCH 0)

SET(DD_VERSION "${DD_VERSION_MAJOR}.${DD_VERSION_MINOR}.${DD_VERSION_PATCH}")

# Get the current working branch
execute_process(
  COMMAND git rev-parse --abbrev-ref HEAD
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  OUTPUT_VARIABLE DD_VERSION_BRANCH
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

# Get the commit hash of the working branch
execute_process(
  COMMAND git rev-parse HEAD
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  OUTPUT_VARIABLE DD_VERSION_COMMIT_HASH
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

# Get the `git describe` string
execute_process(
  COMMAND git describe --always
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  OUTPUT_VARIABLE DD_VERSION_GIT_DESCRIBE
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
