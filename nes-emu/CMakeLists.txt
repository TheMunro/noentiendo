cmake_minimum_required (VERSION 3.16)

set(NES_EMU_INCLUDES
	"inc/bitfield.hpp"
	"inc/nes.hpp"
	"inc/cpu.hpp"
	"inc/bus.hpp"
	"inc/cartridge.hpp"
	"inc/instructions.hpp"
	"inc/mapper.hpp"
	"inc/mapper_000.hpp"
)

set(NES_EMU_SOURCES

	"src/cpu.cpp"
	"src/bus.cpp"
	"src/cartridge.cpp"
	"src/mapper.cpp"
	"src/mapper_000.cpp"
)

# Add source to this project's executable.
add_library (nes-emu ${NES_EMU_INCLUDES} ${NES_EMU_SOURCES})

set_property(TARGET nes-emu PROPERTY CXX_STANDARD 17)

target_include_directories(${CMAKE_PROJECT_NAME}
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/inc"
    PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
)