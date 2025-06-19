cmake_minimum_required(VERSION 3.25)
project(mesh-kernel
    VERSION 0.0.1
    DESCRIPTION "App to compute polygon mesh kernels"
    LANGUAGES CXX)

# ===============================================
# global settings

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# ===============================================
# options

option(MK_TBB_ENABLED "Enable TBB" OFF)
option(MK_MOLD_LINKER_ENABLED "if ON, the mold linker is enabled" ON)

# ===============================================
# compiler and linker flags

set(COMMON_COMPILER_FLAGS "")
set(COMMON_LINKER_FLAGS "")

if (MSVC)
    list(APPEND COMMON_COMPILER_FLAGS /MP)
    list(APPEND COMMON_COMPILER_FLAGS /arch:AVX2) # Added for BMI2 support

    if (MK_ENABLE_WERROR)
        list(APPEND COMMON_COMPILER_FLAGS /WX)
    endif()
else()
    list(APPEND COMMON_COMPILER_FLAGS -Wall -Wextra)
    
    # Check for x86/x64 architecture before adding BMI2 flag
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64|i[3-6]86")
        list(APPEND COMMON_COMPILER_FLAGS -mbmi2) # Added for BMI2 support on x86
    endif()

    if (MK_ENABLE_WERROR)
        list(APPEND COMMON_COMPILER_FLAGS -Werror)
    endif()
endif()

# ===============================================
# Bin dir

if(MSVC)
    set(BIN_DIR ${CMAKE_SOURCE_DIR}/bin)
elseif(CMAKE_BUILD_TYPE STREQUAL "")
    set(BIN_DIR ${CMAKE_SOURCE_DIR}/bin/Default)
else()
    set(BIN_DIR ${CMAKE_SOURCE_DIR}/bin/${CMAKE_BUILD_TYPE})
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BIN_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${BIN_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO ${BIN_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${BIN_DIR})
set(GLOW_BIN_DIR ${CMAKE_SOURCE_DIR}/bin)

# ===============================================
# disable glfw additionals

option(GLFW_BUILD_EXAMPLES  OFF)
option(GLFW_BUILD_TESTS  OFF)
option(GLFW_BUILD_DOCS  OFF)
option(GLFW_INSTALL  OFF)

# ===============================================
# add submodules

# Core dependencies (available and working)
add_subdirectory(extern/clean-core)
add_subdirectory(extern/clean-ranges)
add_subdirectory(extern/typed-geometry)
add_subdirectory(extern/reflector)
add_subdirectory(extern/rich-log)
add_subdirectory(extern/polymesh)
add_subdirectory(extern/CLI11)

# GUI dependencies (commented out due to network access issues)
# add_subdirectory(extern/reflector)
# add_subdirectory(extern/glfw)
# add_subdirectory(extern/glow)
# add_subdirectory(extern/imgui-lean)
# add_subdirectory(extern/glow-extras)
# add_subdirectory(extern/nexus)
# add_subdirectory(extern/cpp-utils)
# add_subdirectory(extern/babel-serializer)
# add_subdirectory(extern/ctracer)

# ===============================================
# setup sources
file(GLOB_RECURSE SOURCES
    "src/*.cc"
    "src/*.hh"
)

# Remove GUI-dependent files when building without GUI support
list(REMOVE_ITEM SOURCES 
    "${CMAKE_CURRENT_SOURCE_DIR}/src/core/kernel-app.cc"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/core/kernel-app.hh"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/core/kernel-plane-cut.cc"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/core/kernel-plane-cut.hh"
)

# Remove imgui directory entirely
file(GLOB_RECURSE IMGUI_SOURCES "src/imgui/*")
list(REMOVE_ITEM SOURCES ${IMGUI_SOURCES})

# Remove rendering directory entirely  
file(GLOB_RECURSE RENDERING_SOURCES "src/rendering/*")
list(REMOVE_ITEM SOURCES ${RENDERING_SOURCES})

# group sources according to folder structure
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${SOURCES})

# ===============================================
# add executable

# remove main.cc, readd later. This allows us to have a separate test executable
list(REMOVE_ITEM SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/main.cc")
add_library(${PROJECT_NAME}_lib STATIC ${SOURCES})

# ===============================================
# Include directories (external as SYSTEM)
target_include_directories(${PROJECT_NAME}_lib SYSTEM PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/extern/eigen)
target_include_directories(${PROJECT_NAME}_lib SYSTEM PRIVATE "extern/cli11")

# Include your source directory normally
target_include_directories(${PROJECT_NAME}_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)

# ===============================================
# linked libraries

target_link_libraries(${PROJECT_NAME}_lib
    PUBLIC
        clean-core
        clean-ranges
        typed-geometry
        reflector
        rich-log
        polymesh
        CLI11::CLI11
        ${COMMON_LINKER_FLAGS}
)

if(MK_TBB_ENABLED)
    message("[mesh-kernel] TBB enabled")
    find_package(TBB REQUIRED)
    target_link_libraries(${PROJECT_NAME}_lib PUBLIC TBB::tbb)
    target_compile_definitions(${PROJECT_NAME}_lib PUBLIC MK_TBB_ENABLED)
endif()

# ===============================================
# disable mold on nixos

if(NOT MK_MOLD_LINKER_ENABLED AND NOT MSVC)
    message(STATUS "Disable mold linker")
    set(GLOW_USE_MOLD_LINKER OFF CACHE BOOL "Disable mold linker" FORCE)
    set(CC_LINKER_MOLD OFF CACHE BOOL "Disable mold linker" FORCE)
endif()

# ===============================================
# Compiler options

# Apply common flags for your executable (warnings enabled)
target_compile_options(${PROJECT_NAME}_lib PRIVATE ${COMMON_COMPILER_FLAGS})  # Apply bmi flag to target

# GUI application disabled due to missing dependencies
# add_executable(${PROJECT_NAME} "src/main.cc")
# target_link_libraries(${PROJECT_NAME} PRIVATE ${PROJECT_NAME}_lib)

# Add EMBER CSG executable
add_executable(ember-csg "src/ember-main.cc")
target_link_libraries(ember-csg PRIVATE ${PROJECT_NAME}_lib)

# Add EMBER test executable
add_executable(ember-test "src/ember-test.cc")
target_link_libraries(ember-test PRIVATE ${PROJECT_NAME}_lib)
