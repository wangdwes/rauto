
cmake_minimum_required(VERSION 2.8)
project(navigator)

find_package(PkgConfig REQUIRED)
pkg_check_modules(SBPL REQUIRED sbpl)
include_directories(src ${SBPL_INCLUDE_DIRS})
link_directories(${SBPL_LIBRARY_DIRS})

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/..)

add_library(navigator SHARED src/navigator.cc)
target_include_directories(navigator PUBLIC ${SBPL_INCLUDE_DIRS}) 
