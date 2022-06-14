# ====================================================================================================================================
# @file       CMakeLists.txt
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Tuesday, 1st March 2022 4:30:53 pm
# @modified   Monday, 13th June 2022 10:41:32 am
# @project    engineering-thesis
# @brief      Driver plugin for the IMU EtherCAT slave
#    
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Project ============================================================ #

# CMake version
cmake_minimum_required(VERSION 3.8)
# Project header
project(velmwheel_imu_driver)

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
    imu_ethercat
    velmwheel_ethercat_driver
    velmwheel_imu_driver_msgs
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
        src/bias_callbacks.cpp
        src/digital_filter_callbacks.cpp
        src/general_callbacks.cpp
        src/gyro_range_callbacks.cpp
        src/imu_driver.cpp
        src/measurement_callbacks.cpp

    # Dependencies
    AMENT_DEPENDENCIES ${LIB_DEPENDENCIES}
    
)
target_compile_options(${PROJECT_NAME} PRIVATE -O2)
# Export plugin description file
pluginlib_export_plugin_description_file(${PROJECT_NAME}
    plugins.xml
)

# ======================================================== Source (Python) ========================================================= #

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
