# ====================================================================================================================================
# @file       CMakeLists.txt
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Tuesday, 1st March 2022 4:30:53 pm
# @modified   Wednesday, 25th May 2022 11:16:52 pm
# @project    engineering-thesis
# @brief      CMake file for `velmwheel_model` package
# 
# 
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Project ============================================================ #

# CMake version
cmake_minimum_required(VERSION 3.8)
# Project header
project(velmwheel_model)
# ========================================================== Dependencies ========================================================== #

# Ament dependency (always present)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
# Build dependencies
find_package(geometry_msgs REQUIRED)
find_package(velmwheel_msgs REQUIRED)

# ============================================================= Source ============================================================= #

# List dependencies
list(APPEND dependencies
    geometry_msgs
    velmwheel_msgs
)

# Add target library
add_library(${PROJECT_NAME} INTERFACE)
# Link dependencies
ament_target_dependencies(${PROJECT_NAME}
    INTERFACE
        ${dependencies}
)
# Add include directories to the target
target_include_directories(${PROJECT_NAME}
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# Install targets
install(
    TARGETS
        ${PROJECT_NAME}
    EXPORT
        ${PROJECT_NAME}Targets
    INCLUDES DESTINATION
        include
)

# Export targets
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
# Export dependencies
ament_export_dependencies(${dependencies})

# ========================================================= Share installs ========================================================= #

# Install headers
install(
    DIRECTORY
        include/
    DESTINATION
        include
)

# Install shared data
install(
    DIRECTORY
        models
        urdf
    DESTINATION
        share/${PROJECT_NAME}
)

# ============================================================== Build ============================================================= #

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Build package using ament
ament_package()
