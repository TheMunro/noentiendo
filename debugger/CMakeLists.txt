cmake_minimum_required (VERSION 3.16)

find_package(Vulkan)

if (NOT Vulkan_FOUND)
    message(STATUS "Warning - Cannot find Vulkan Vulkan_PATH=${Vulkan_PATH}")
    return()
endif()

find_package(SDL2)

if (NOT SDL2_FOUND)
    message(STATUS "Warning - Cannot find SDL2 SDL2_PATH=${SDL2_PATH}")
    return()
endif()

include(FetchContent)

set(FETCHCONTENT_BASE_DIR "${CMAKE_CURRENT_BINARY_DIR}")

message(STATUS "Fetching IMGUI...")
FetchContent_Declare(imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui
    GIT_TAG        v1.79
    GIT_SHALLOW    TRUE
)
FetchContent_GetProperties(imgui)
if (NOT imgui_POPULATED)
    FetchContent_Populate(imgui)

    add_library(imgui
        "${imgui_SOURCE_DIR}/imgui.cpp"
        "${imgui_SOURCE_DIR}/imgui_draw.cpp"
        "${imgui_SOURCE_DIR}/imgui_widgets.cpp"
        "${imgui_SOURCE_DIR}/imgui_demo.cpp")

    target_include_directories(imgui SYSTEM PUBLIC "${imgui_SOURCE_DIR}")
endif ()

message(STATUS "Fetched IMGUI")

set(DEBUGGER_INCLUDES
    "inc/debugger.hpp"
)

set(NES_EMU_SOURCES
    "src/debugger.cpp"
	"src/main.cpp"
)

# Add source to this project's executable.
add_executable (debugger ${NES_EMU_INCLUDES} ${NES_EMU_SOURCES})
target_link_libraries(debugger nes-emu imgui)

if (WIN32)
    add_custom_target(sdl2_copy
        COMMAND ${CMAKE_COMMAND} -E copy
                "${SDL2_LIBRARY}/../SDL2.dll"
                "${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>/SDL2.dll"
    )
    add_dependencies(debugger sdl2_copy)
endif()

if(WIN32)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /SUBSYSTEM:WINDOWS /ENTRY:mainCRTStartup")
endif()

set_property(TARGET debugger 
	PROPERTY CXX_STANDARD 17)

target_include_directories(${CMAKE_PROJECT_NAME}
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/inc"
    PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
)
