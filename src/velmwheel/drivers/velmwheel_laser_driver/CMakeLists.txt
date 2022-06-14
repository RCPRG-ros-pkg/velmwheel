# ====================================================================================================================================
# @file       CMakeLists.txt
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Tuesday, 1st March 2022 4:30:53 pm
# @modified   Wednesday, 25th May 2022 4:44:01 pm
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

# Ament dependency (always present)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

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
