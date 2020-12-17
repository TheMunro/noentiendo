cmake_minimum_required (VERSION 3.16)

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

    set_target_properties(imgui PROPERTIES FOLDER dependencies) 
    set_target_properties(imgui PROPERTIES LINKER_LANGUAGE CXX)

endif ()

message(STATUS "Fetched IMGUI")

set(DEBUGGER_INCLUDES
)

set(NES_EMU_SOURCES

	"src/main.cpp"
)

# Add source to this project's executable.
add_executable (debugger ${NES_EMU_INCLUDES} ${NES_EMU_SOURCES})
target_link_libraries(debugger nes-emu imgui)

set_property(TARGET debugger 
	PROPERTY CXX_STANDARD 17)

target_include_directories(${CMAKE_PROJECT_NAME}
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/inc"
    PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
)
