cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME balance_infantry)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/DSP/Lib/GCC
)

# Add sources to executable
# Collect all user sources under Bsp/ and Robot/ (CONFIGURE_DEPENDS re-globs on rebuild)
file(GLOB_RECURSE USER_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_SOURCE_DIR}/Bsp/*.c
    ${CMAKE_SOURCE_DIR}/Robot/*.c
)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${USER_SOURCES})

# Add include paths
# Every directory containing a user header becomes an include path
file(GLOB_RECURSE USER_HEADERS CONFIGURE_DEPENDS
    ${CMAKE_SOURCE_DIR}/Bsp/*.h
    ${CMAKE_SOURCE_DIR}/Robot/*.h
)
set(USER_INCLUDE_DIRS "")
foreach(_header IN LISTS USER_HEADERS)
    get_filename_component(_header_dir ${_header} DIRECTORY)
    list(APPEND USER_INCLUDE_DIRS ${_header_dir})
endforeach()
list(REMOVE_DUPLICATES USER_INCLUDE_DIRS)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ${USER_INCLUDE_DIRS}
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
    arm_cortexM7lfdp_math
)

# Enable %f / %e support in newlib-nano printf (same as -u _printf_float in the Makefile)
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    "SHELL:-u _printf_float"
)
