cmake_minimum_required(VERSION 2.8.7)
project(cpp_example)

find_package(director REQUIRED)
include(${DIRECTOR_USE_FILE})


include_directories(${DIRECTOR_INCLUDE_DIRS})
add_executable(my_app my_app.cpp)
target_link_libraries(my_app ${DIRECTOR_LIBRARIES})


add_library(my_library SHARED my_library.cpp)
target_link_libraries(my_library ${DIRECTOR_LIBRARIES})
