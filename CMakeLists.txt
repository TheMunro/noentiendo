cmake_minimum_required (VERSION 3.16)

project ("nes-emu")

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# Include sub-projects.
add_subdirectory ("nes-emu")
add_subdirectory ("debugger")
add_subdirectory("dependencies")
