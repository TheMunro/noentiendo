cmake_minimum_required (VERSION 3.16)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_BINARY_DIR}/dependencies/sdl2_cmake_modules-src")
message(STATUS "CMAKE_MODULE_PATH = ${CMAKE_MODULE_PATH}")

project ("noentiendo")

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# Include sub-projects.
add_subdirectory("dependencies")
add_subdirectory ("noentiendo")
add_subdirectory ("debugger")
