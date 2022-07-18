# ====================================================================================================================================
# @file       CMakeLists.txt
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Wednesday, 13th April 2021 12:43:42 am
# @modified   Monday, 11th July 2022 10:24:24 pm
# @project    engineering-thesis
# @brief
#    
#    Build rules for Hilscher's CIFX/netX Toolkit-based CIFX EtherCAT Master
#
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Project ============================================================ #

# CMake version
cmake_minimum_required(VERSION 3.5)
# Project header
project(cifx_ethercat VERSION 1.0.0)

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
    package_common_cpp
    ethercat-lib
    cifx_toolkit
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
        src/cifx/ethercat/slave.cpp
        src/cifx/ethercat/master.cpp
        src/cifx/ethercat/utilities.cpp

    # Includes
    ADDITIONAL_INCLUDES
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/ecm-api>
        $<INSTALL_INTERFACE:include/ecm-api>

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

# ========================================================= Shared installs ======================================================== #

# Install shared resources
install(
    DIRECTORY
        firmware
    DESTINATION
        share/${PROJECT_NAME}
)

# Register ament resources
ament_index_register_resource(firmware CONTENT share/${PROJECT_NAME}/firmware)

# Install Hilscher's documentation
install(
    DIRECTORY
        doc/materials/
    DESTINATION
        share/${PROJECT_NAME}/doc/materials
)

# ============================================================== Build ============================================================= #

ament_package()
