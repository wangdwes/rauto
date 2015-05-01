
cmake_minimum_required(VERSION 2.8)
find_package(PkgConfig REQUIRED)
pkg_check_modules(SBPL REQUIRED sbpl)
include_directories(src ${SBPL_INCLUDE_DIRS})
link_directories(${SBPL_LIBRARY_DIRS})

add_executable(navigator src/navigator.cc src/launcher.cc)
target_link_libraries(navigator ${SBPL_LIBRARIES})
