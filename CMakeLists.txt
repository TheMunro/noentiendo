# CMakeList.txt : Top-level CMake project file, do global configuration
# and include sub-projects here.
#
cmake_minimum_required (VERSION 3.16)

project ("nes-emu")

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# Include sub-projects.
add_subdirectory ("nes-emu")
add_subdirectory("dependencies")
