# ====================================================================================================================================
# @file       CMakeLists.txt
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Wednesday, 13th April 2021 12:43:42 am
# @modified   Tuesday, 5th July 2022 1:05:00 am
# @project    engineering-thesis
# @brief      Build rules for C++ library providing interface class for manaing LMS1xx LIDAR sensor
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Project ============================================================ #

# CMake version
cmake_minimum_required(VERSION 3.5)
# Project header
project(lms1xx VERSION 1.0.0)

# ========================================================== Configuration ========================================================= #

# Compilation proprties
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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
    rclcpp
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
        src/lms1xx/scan_data_buffer.cpp
        src/lms1xx/scan_data_parser.cpp
        src/lms1xx/io_manager.cpp
        src/lms1xx.cpp
    
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
