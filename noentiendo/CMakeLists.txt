cmake_minimum_required (VERSION 3.16)

set(noentiendo_INCLUDES
	"inc/bitfield.hpp"
	"inc/nes.hpp"
	"inc/cpu.hpp"
	"inc/bus.hpp"
	"inc/cartridge.hpp"
	"inc/instructions.hpp"
	"inc/mapper.hpp"
	"inc/mapper_000.hpp"
)

set(noentiendo_SOURCES

	"src/cpu.cpp"
	"src/bus.cpp"
	"src/cartridge.cpp"
	"src/mapper.cpp"
	"src/mapper_000.cpp"
)

# Add source to this project's executable.
add_library (noentiendo ${noentiendo_INCLUDES} ${noentiendo_SOURCES})
target_link_libraries(noentiendo fmt::fmt-header-only spdlog::spdlog_header_only)

set_property(TARGET noentiendo PROPERTY CXX_STANDARD 17)

target_include_directories(noentiendo
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/inc"
    PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
)