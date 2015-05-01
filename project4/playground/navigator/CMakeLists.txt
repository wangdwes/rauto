
cmake_minimum_required(VERSION 2.8)
project(navigator)

find_package(PkgConfig REQUIRED)
pkg_check_modules(SBPL REQUIRED sbpl)
include_directories(src ${SBPL_INCLUDE_DIRS})
link_directories(${SBPL_LIBRARY_DIRS})

add_library(navigator SHARED src/navigator.cc)
target_include_directories(navigator PUBLIC ${SBPL_INCLUDE_DIRS}) 

add_executable(navigator_test src/launcher.cc)
target_link_libraries(navigator_test LINK_PUBLIC navigator ${SBPL_LIBRARIES})
