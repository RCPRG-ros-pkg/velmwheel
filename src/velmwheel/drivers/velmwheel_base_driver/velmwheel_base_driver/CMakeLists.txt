# ====================================================================================================================================
# @file       CMakeLists.txt
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Tuesday, 1st March 2022 4:30:53 pm
# @modified   Friday, 15th July 2022 3:09:11 pm
# @project    engineering-thesis
# @brief      Driver plugin for the Velmwheel base EtherCAT slave
#    
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Project ============================================================ #

# CMake version
cmake_minimum_required(VERSION 3.8)
# Project header
project(velmwheel_base_driver)

# ========================================================== Configuration ========================================================= #

# Add extra warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

# ========================================================== Dependencies ========================================================== #

# List dependencies 
list(APPEND LIB_DEPENDENCIES
    pluginlib
    node_common_cpp
    tf2_ros
    tf2_geometry_msgs
    gold_dc_whistle
    velmwheel_ethercat_driver
    velmwheel_model
    velmwheel_msgs
    velmwheel_base_driver_msgs
    sensor_msgs
)

# Utilities dependencies
find_package(cmake_common REQUIRED)
# Find dependencies
find_packages(

    # Ament dependencies
    ament_cmake
    ament_cmake_python
    # Common dependencies
    ${LIB_DEPENDENCIES}
    
)

# ============================================================= Source ============================================================= #

# Add target
add_ament_library(

    # Library name
    LIBRARY_NAME ${PROJECT_NAME}

    # Library sources
    SOURCES
        src/base_driver.cpp
        src/service_callbacks.cpp
        src/synchronous_callbacks.cpp

    # Dependencies
    AMENT_DEPENDENCIES ${LIB_DEPENDENCIES}
    
)

# Export plugin description file
pluginlib_export_plugin_description_file(velmwheel_ethercat_driver plugins.xml)

# ===================================================== Source (Python nodes) ====================================================== #

# Install Python executables
install(
    
    # Auxiliary ROS nodes
    PROGRAMS
        
        ${PROJECT_NAME}/enable.py
        ${PROJECT_NAME}/get_state.py
        ${PROJECT_NAME}/reset_fault.py
        
    # Destination
    DESTINATION lib/${PROJECT_NAME}
    
)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# ========================================================= Share installs ========================================================= #

# Install shared data
install(
    DIRECTORY
        config
    DESTINATION
        share/${PROJECT_NAME}
)
# ============================================================== Build ============================================================= #

# Build package using ament
ament_package()
