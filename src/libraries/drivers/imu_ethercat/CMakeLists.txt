# ====================================================================================================================================
# @file       CMakeLists.txt
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Wednesday, 13th April 2021 12:43:42 am
# @modified   Wednesday, 1st June 2022 2:45:31 pm
# @project    engineering-thesis
# @brief      Build rules for C++ library providing hardware-independent implementation of the EtherCAT driver for proprietary IMU 
#             ssensor board by B. Kaczor
#    
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Project ============================================================ #

# CMake version
cmake_minimum_required(VERSION 3.5)
# Project header
project(imu_ethercat VERSION 1.0.0)

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
    ethercat
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
    # Library type
    LIBRARY_TYPE INTERFACE

    # Dependencies
    AMENT_DEPENDENCIES ${LIB_DEPENDENCIES}
    
)

# =============================================================== Docs ============================================================= #

# Build doc
if(BUILD_DOC)
    add_subdirectory(doc)
endif()

# ============================================================== Build ============================================================= #

ament_package()
