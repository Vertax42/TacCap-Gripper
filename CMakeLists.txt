cmake_minimum_required(VERSION 3.20)
project(taccap_gripper VERSION 0.0.1 LANGUAGES CXX)

# ==============================================================================
# Standards & global settings
# ==============================================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

# ==============================================================================
# Options
# ==============================================================================
option(TACCAP_BUILD_PYTHON   "Build pybind11 module (xense.taccap)" ON)
option(TACCAP_BUILD_EXAMPLES "Build C++ examples"                   OFF)

# ==============================================================================
# Vendored libxensesdk in lite mode
# ==============================================================================
# Force-set the lite knob BEFORE the submodule's CMakeLists.txt runs so it
# never searches for ONNX / CUDA / MIGraphX.
set(XENSE_LITE_BUILD ON CACHE BOOL "TacCap consumes libxense in lite mode" FORCE)

if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/libxensesdk/CMakeLists.txt")
    message(FATAL_ERROR
        "third_party/libxensesdk is missing. Run:\n"
        "  git submodule update --init --recursive")
endif()

add_subdirectory(third_party/libxensesdk EXCLUDE_FROM_ALL)

# ==============================================================================
# Generated version header
# ==============================================================================
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cpp/include/taccap/version.hpp.in
    ${CMAKE_CURRENT_BINARY_DIR}/include/taccap/version.hpp
    @ONLY
)

# ==============================================================================
# taccap_core (header-only / interface library)
# ==============================================================================
# Public TacCap API. For now this is just a namespace alias header re-exporting
# the libxense lite subset under xense::taccap::. New TacCap-native types
# (Camera, IMU, Encoder, Motor, LeaderGripper, FollowerGripper) will be added
# here in subsequent steps.
add_library(taccap_core INTERFACE)
target_include_directories(taccap_core INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/cpp/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
)
target_link_libraries(taccap_core INTERFACE xensesdk)

# ==============================================================================
# Python bindings & examples
# ==============================================================================
if(TACCAP_BUILD_PYTHON)
    add_subdirectory(python)
endif()

if(TACCAP_BUILD_EXAMPLES)
    add_subdirectory(cpp/examples)
endif()

# ==============================================================================
# Summary
# ==============================================================================
message(STATUS "")
message(STATUS "taccap-gripper configuration:")
message(STATUS "  Version:           ${PROJECT_VERSION}")
message(STATUS "  Build type:        ${CMAKE_BUILD_TYPE}")
message(STATUS "  C++ standard:      ${CMAKE_CXX_STANDARD}")
message(STATUS "  Python bindings:   ${TACCAP_BUILD_PYTHON}")
message(STATUS "  C++ examples:      ${TACCAP_BUILD_EXAMPLES}")
message(STATUS "  libxensesdk mode:  LITE (XENSE_LITE_BUILD=ON)")
message(STATUS "")
