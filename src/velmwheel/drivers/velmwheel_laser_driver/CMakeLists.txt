# ====================================================================================================================================
# @file       CMakeLists.txt
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Tuesday, 1st March 2022 4:30:53 pm
# @modified   Tuesday, 5th July 2022 6:10:15 am
# @project    engineering-thesis
# @brief      CMake file for `velmwheel_laser_driver` package
#    
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Project ============================================================ #

# CMake version
cmake_minimum_required(VERSION 3.8)
# Project header
project(velmwheel_laser_driver)

# ========================================================== Dependencies ========================================================== #

# Librarie's dependencies
list(APPEND DEPENDENCIES
    lms1xx
    rclcpp
    rclcpp_components
    node_common_cpp
    sensor_msgs
    tf2_eigen
)

# Common dependencies
find_package(cmake_common REQUIRED)
# Ament dependency (always present)
find_packages(

    # Ament dependencies
    ament_cmake
    ament_cmake_python
    # Find dependencies
    ${DEPENDENCIES}

)

# ============================================================= Source ============================================================= #

# Set lib target names
STRING(REPLACE "velmwheel_" "" LIB_TARGET ${PROJECT_NAME})
# Set executable target names
set(EXEC_TARGET ${PROJECT_NAME})

# Add node
add_ros_component_node(

    # Library target
    LIBRARY_NAME ${LIB_TARGET}
    # Plugin target
    PLUGIN_NAME "velmwheel::LaserDriver"
    # Executable target
    EXECUTABLE_NAME ${EXEC_TARGET}

    # Sources
    SOURCES
        src/laser_driver.cpp
        src/callbacks.cpp
        src/configuration.cpp

    # Ament dependencies
    AMENT_DEPENDENCIES ${DEPENDENCIES}
    
)

# ========================================================== Python nodes ========================================================== #

# Install Python executables
install(
    PROGRAMS
        ${PROJECT_NAME}/device_finder.py
    DESTINATION
        lib/${PROJECT_NAME}
)

# ========================================================= Share installs ========================================================= #

# Install shared data
install(
    DIRECTORY
        config
        launch
    DESTINATION
        share/${PROJECT_NAME}
)

# ============================================================== Build ============================================================= #

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Build package using ament
ament_package()
