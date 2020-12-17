include(FetchContent)

set(FETCHCONTENT_BASE_DIR "${CMAKE_CURRENT_BINARY_DIR}")

# fmt
message(STATUS "Fetching fmt...")
FetchContent_Declare(fmt
    GIT_REPOSITORY https://github.com/fmtlib/fmt
    GIT_TAG        7.1.3
    GIT_SHALLOW    TRUE
    CMAKE_ARGS
        -DFMT_DOC=OFF
        -DFMT_INSTALL=OFF
        -DFMT_TEST=OFF
)
FetchContent_GetProperties(fmt)
if (NOT fmt_POPULATED)
    FetchContent_Populate(fmt)
    add_subdirectory("${fmt_SOURCE_DIR}" "${fmt_BINARY_DIR}" EXCLUDE_FROM_ALL)
endif ()
message(STATUS "Fetched fmt")


# spdlog
message(STATUS "Fetching spdlog...")
FetchContent_Declare(spdlog
    GIT_REPOSITORY https://github.com/gabime/spdlog
    GIT_TAG        v1.8.2
    GIT_SHALLOW    TRUE
)
FetchContent_GetProperties(spdlog)
if (NOT spdlog_POPULATED)
    FetchContent_Populate(spdlog)
    add_subdirectory("${spdlog_SOURCE_DIR}" "${spdlog_BINARY_DIR}" EXCLUDE_FROM_ALL)
endif ()
message(STATUS "Fetched spdlog")


# catch2
message(STATUS "Fetching Catch2...")
FetchContent_Declare(catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2
    GIT_TAG        v2.13.3
    GIT_SHALLOW    TRUE
)
FetchContent_GetProperties(catch2)
if (NOT catch2_POPULATED)
    FetchContent_Populate(catch2) 
    add_subdirectory("${catch2_SOURCE_DIR}" "${catch2_BINARY_DIR}" EXCLUDE_FROM_ALL)
endif ()
message(STATUS "Fetched Catch2")

message(STATUS "Fetching sdl2-cmake-modules...")
FetchContent_Declare(sdl2_cmake_modules
    GIT_REPOSITORY https://github.com/aminosbh/sdl2-cmake-modules
    GIT_SHALLOW    TRUE
)
if (NOT sdl2_cmake_modules_POPULATED)
    FetchContent_Populate(sdl2_cmake_modules)
endif()
message(STATUS "Fetched sdl2-cmake-modules")