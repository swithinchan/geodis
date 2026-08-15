cmake_minimum_required(VERSION 3.21)
project(geodis
    VERSION 1.0.0
    DESCRIPTION "3D Pedestrian Network Distance Calculator & Fieldwork Assignment Engine"
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ── Build options ─────────────────────────────────────────────────────────
option(GEODIS_USE_TBB "Enable TBB parallelism" ON)

# ── Find packages ─────────────────────────────────────────────────────────
find_package(Boost REQUIRED COMPONENTS system)
find_package(TBB QUIET)

if(TBB_FOUND AND GEODIS_USE_TBB)
    message(STATUS "TBB found: parallel Dijkstra enabled")
else()
    message(STATUS "TBB not found — sequential execution")
endif()

# ── Platform detection ────────────────────────────────────────────────────
if(WIN32)
    add_compile_definitions(GEODIS_PLATFORM_WINDOWS)
endif()

# ── Compiler flags ────────────────────────────────────────────────────────
if(MSVC)
    add_compile_options(/W4 /O2 /arch:AVX2 /utf-8 /wd4996)
else()
    add_compile_options(-Wall -Wextra -O3 -march=native)
    add_compile_options(-Wno-sign-compare -Wno-unused-parameter)
endif()

# ── Library: geodis_core ──────────────────────────────────────────────────
add_library(geodis_core STATIC
    src/spatial/rtree_index.cpp
    src/graph/network_graph.cpp
    src/routing/dijkstra.cpp
    src/assignment/assignment_engine.cpp
)

target_include_directories(geodis_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${Boost_INCLUDE_DIRS}
)

target_link_libraries(geodis_core PUBLIC
    Boost::system
)

if(TBB_FOUND AND GEODIS_USE_TBB)
    target_link_libraries(geodis_core PUBLIC TBB::tbb)
    target_compile_definitions(geodis_core PUBLIC GEODIS_HAS_TBB)
endif()

# ── Executable: geodis-viewer (OpenGL 3D viewer) ───────────────────────────
option(GEODIS_BUILD_VIEWER "Build OpenGL 3D viewer" ON)

if(GEODIS_BUILD_VIEWER)
    find_package(OpenGL REQUIRED)
    find_package(glfw3 QUIET)

    add_executable(geodis-viewer
        src/viewer/viewer.cpp
    )
    target_include_directories(geodis-viewer PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/external
    )
    target_link_libraries(geodis-viewer PRIVATE
        OpenGL::GL
    )
    if(glfw3_FOUND)
        target_link_libraries(geodis-viewer PRIVATE glfw3)
    elseif(WIN32)
        target_link_libraries(geodis-viewer PRIVATE glfw3)
    else()
        # Try to find glfw on Linux
        find_library(GLFW_LIB NAMES glfw glfw3 libglfw.so.3)
        if(GLFW_LIB)
            target_link_libraries(geodis-viewer PRIVATE ${GLFW_LIB})
            message(STATUS "GLFW found: ${GLFW_LIB}")
        else()
            message(WARNING "GLFW not found — viewer won't link. Install glfw-devel.")
        endif()
    endif()
    target_link_libraries(geodis-viewer PRIVATE m)
    set_target_properties(geodis-viewer PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    )
    install(TARGETS geodis-viewer RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()

# ── Executable: geodis ────────────────────────────────────────────────────
add_executable(geodis src/main.cpp)
target_link_libraries(geodis PRIVATE geodis_core)

# ── Install ───────────────────────────────────────────────────────────────
include(GNUInstallDirs)
install(TARGETS geodis RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(TARGETS geodis_core ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY src/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/geodis
    FILES_MATCHING PATTERN "*.hpp"
)

# ── Summary ───────────────────────────────────────────────────────────────
message(STATUS "geodis v${PROJECT_VERSION} | C++${CMAKE_CXX_STANDARD} | ${CMAKE_BUILD_TYPE}")
