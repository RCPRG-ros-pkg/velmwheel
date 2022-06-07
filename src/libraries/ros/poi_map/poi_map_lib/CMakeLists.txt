# ====================================================================================================================================
# @file       CMakeLists.txt
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Tuesday, 1st March 2022 4:30:53 pm
# @modified   Wednesday, 25th May 2022 5:43:57 pm
# @project    engineering-thesis
# @brief      Minimal CMake file for the 'poi_map_lib' package
#    
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Project ============================================================ #

# CMake version
cmake_minimum_required(VERSION 3.8)
# Project header
project(poi_map_lib)

# ========================================================== Configuration ========================================================= #

# Add extra warnings
add_compile_options(-Wall -Wextra -Wpedantic)

# Configure C++17 (instead of original C++14) to meet rclcpp requirement for std::variant
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Options
option(BUILD_DOC "If TRUE documentation will be built" ON)

# ========================================================== Dependencies ========================================================== #

# Librarie's link dependencies
list(APPEND LINK_DEPENDENCIES
    rclcpp
    std_msgs
    geometry_msgs
    visualization_msgs
    poi_map_msgs
    tf2
)

# Librarie's dependencies
list(APPEND DEPENDENCIES

    # Link dependencies
    ${LINK_DEPENDENCIES}
    # Additional dependencies
    eigen3_cmake_module
    
)

# Common dependencies
find_package(cmake_common REQUIRED)
# Ament dependency (always present)
find_packages(

    # Ament dependencies
    ament_cmake
    # Find dependencies
    ${DEPENDENCIES}

)

# ======================================================== Source (Library) ======================================================== #

# Add target
add_ament_library(

    # Library name
    LIBRARY_NAME ${PROJECT_NAME}

    # Library sources
    SOURCES
        src/map_loader.cpp
        src/map_builder.cpp
        src/map_visualizer.cpp

    # Dependencies
    AMENT_DEPENDENCIES ${LINK_DEPENDENCIES}
    
)

# =============================================================== Docs ============================================================= #

# Build doc
if(BUILD_DOC)
    add_subdirectory(doc)
endif()

# ============================================================== Build ============================================================= #

# Build package using ament
ament_package()
