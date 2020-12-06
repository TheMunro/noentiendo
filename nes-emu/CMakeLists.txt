# CMakeList.txt : CMake project for nes-emu, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.16)

set(NES_EMU_INCLUDES
	"inc/cpu.hpp"
	"inc/bus.hpp"
)

set(NES_EMU_SOURCES

	"src/bus.cpp"

	"src/main.cpp"
)

# Add source to this project's executable.
add_executable (nes-emu ${NES_EMU_INCLUDES} ${NES_EMU_SOURCES})

target_include_directories(${CMAKE_PROJECT_NAME}
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/inc"
    PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
)

# TODO: Add tests and install targets if needed.
