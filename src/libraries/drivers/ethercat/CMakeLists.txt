# ====================================================================================================================================
# @file       CMakeLists.txt
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Wednesday, 13th April 2021 12:43:42 am
# @modified   Thursday, 9th June 2022 2:28:03 am
# @project    engineering-thesis
# @brief      Build rules for C++ EtherCAT library
#    
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Project ============================================================ #

# CMake version
cmake_minimum_required(VERSION 3.5)
# Project header
project(ethercat VERSION 1.0.0)

# ========================================================== Configuration ========================================================= #

# Compilation proprties
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Options
option(BUILD_DOC "If TRUE documentation will be built" ON)

# Compilation options
add_compile_options(
    -Wpedantic
    -Wextra
    -Wall
    -Wno-missing-field-initializers
)

# ========================================================== Dependencies ========================================================== #

# Linkable dependencies
list(APPEND LIB_DEPENDENCIES
    Boost
)

# Common dependencies
find_package(cmake_common REQUIRED)
# Dependencies
find_packages(

    # Ament dependency (always present)
    ament_cmake
    # Find dependencies
    ${LIB_DEPENDENCIES}

)
# ============================================================= Source ============================================================= #

# Add target
add_ament_library(

    # Library name
    LIBRARY_NAME ${PROJECT_NAME}

    # Library sources
    SOURCES 
        # Types sources
        src/ethercat/types/builtin.cpp
        src/ethercat/types/type.cpp
        # ENI sources
        src/ethercat/eni/configuration/configuration.cpp
        src/ethercat/eni/process_image/process_image.cpp
        src/ethercat/eni/process_image/variable.cpp
        src/ethercat/eni/slave/slave.cpp
        src/ethercat/eni/slave/pdo.cpp

    # Dependencies
    AMENT_DEPENDENCIES ${LIB_DEPENDENCIES}
    
)

# ============================================================== Tests ============================================================= #

# Test builds
if(BUILD_TESTING)
    add_subdirectory(tests)
endif()

# =============================================================== Docs ============================================================= #

# Build doc
if(BUILD_DOC)
    add_subdirectory(doc)
endif()

# ============================================================== Build ============================================================= #

ament_package()
