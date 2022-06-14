# ====================================================================================================================================
# @file       CMakeLists.txt
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Wednesday, 13th April 2021 12:43:42 am
# @modified   Wednesday, 25th May 2022 9:51:04 pm
# @project    engineering-thesis
# @brief
#    
#    Build rules for Hilscher's CIFX/netX Toolkit-base CIFX card driver
#    
# @note CIFX/netX Toolkit V2.6.0 uses unaligned pointers of the `packed` structures so `no-address-of-packed-member` compilation
#    flag  is used to silence warnings
#
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Project ============================================================ #

# CMake version
cmake_minimum_required(VERSION 3.5)
# Project header
project(cifx_toolkit VERSION 1.0.0)

# ========================================================== Configuration ========================================================= #

# Enabled tests' building
option(BUILD_TESTING "Enables tests' building" ON)

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
    -Wno-address-of-packed-member
    -Wpedantic
    -Wextra
    -Wall
    -Og
)

# ==================================================== Compilation configuration =================================================== #

# # Big Endian platform
# list(APPEND CIFX_TOOLKIT_PUBLIC_DEFS -DCIFX_TOOLKIT_BIGENDIAN)

# # Extended parameters check inside the Toolkit
# list(APPEND CIFX_TOOLKIT_PUBLIC_DEFS -DCIFX_TOOLKIT_PARAMETER_CHECK)

# # Device's time setting at startup
# list(APPEND CIFX_TOOLKIT_PUBLIC_DEFS -DCIFX_TOOLKIT_TIME)

# DMA option for CIFX C Toolkit
list(APPEND CIFX_TOOLKIT_PUBLIC_DEFS -DCIFX_TOOLKIT_DMA)

# ========================================================== Dependencies ========================================================== #

# Common dependencies
find_package(cmake_common REQUIRED)
# Dependencies
find_packages(

    # Ament dependency (always present)
    ament_cmake
    # Find dependencies
    package_common_cpp
    range-v3
    libuio

)

# ============================================================= Source ============================================================= #

# Add target
add_ament_library(

    # Library name
    LIBRARY_NAME ${PROJECT_NAME}

    # Library sources
    SOURCES
        src/driver/cifxDriver.c
        src/driver/cifxDevice.c
        src/driver-cpp/cifx/channel/channel.cpp
        src/driver-cpp/cifx/device/device.cpp
        src/driver-cpp/cifx/channel/mailbox/mailbox.cpp
        src/driver-cpp/cifx/channel/process_data/process_data.cpp
        src/driver-cpp/cifx/utilities/utilities.cpp
        src/os-dependent/OS_Common.c
        src/os-dependent/OS_CosPolling.c
        src/os-dependent/OS_Event.c
        src/os-dependent/OS_File.c
        src/os-dependent/OS_Includes.c
        src/os-dependent/OS_Irq.c
        src/os-dependent/OS_Lock.c
        src/os-dependent/OS_Mem.c
        src/os-dependent/OS_Mutex.c
        src/os-dependent/OS_Init.c
        src/os-dependent/OS_Pci.c
        src/os-dependent/OS_Strings.c
        src/os-dependent/OS_Time.c
        src/toolkit/cifXDownload.c
        src/toolkit/cifXEndianess.c
        src/toolkit/cifXFunctions.c
        src/toolkit/cifXHWFunctions.c
        src/toolkit/cifXInit.c
        src/toolkit/cifXInterrupt.c
        src/toolkit/Hilmd5.c
        src/toolkit/netX5x_hboot.c
        src/toolkit/netX5xx_hboot.c
        src/toolkit/netX90_netX4x00.c
        src/user/user.c

    # Includes
    OVERRIDE_INCLUDES
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/cifx-api>
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/driver>
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/driver-cpp>
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/hilscher-definitions>
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/os-dependent>
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/toolkit>
            $<INSTALL_INTERFACE:include/cifx-api>
            $<INSTALL_INTERFACE:include/driver>
            $<INSTALL_INTERFACE:include/driver-cpp>
            $<INSTALL_INTERFACE:include/hilscher-definitions>
            $<INSTALL_INTERFACE:include/os-dependent>
            $<INSTALL_INTERFACE:include/toolkit>
        PRIVATE
            src/os-dependent

    # Dependencies
    AMENT_DEPENDENCIES package_common_cpp
    # Depdnencies type
    AMENT_TARGET_DEPENDENCIES_TYPE PUBLIC
    
)

# Add build options
target_compile_definitions(${PROJECT_NAME}
    PUBLIC
        ${CIFX_TOOLKIT_PUBLIC_DEFS}
)

# Link additional depdndencies
target_link_libraries(${PROJECT_NAME}
    PUBLIC
        range-v3
    PRIVATE
        pthread
        libuio::uio
)

# Export public dependencies
ament_export_dependencies(range-v3)

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

# Install bootloaders
install(
    DIRECTORY
        bootloaders
    DESTINATION
        share/${PROJECT_NAME}
)

# Register ament resources
ament_index_register_resource(bootloaders CONTENT share/${PROJECT_NAME}/bootloaders)

# Install Hilsher documentation
install(
    DIRECTORY
        doc/materials/
    DESTINATION
        share/${PROJECT_NAME}/doc/materials
)

# ============================================================== Build ============================================================= #

ament_package()
