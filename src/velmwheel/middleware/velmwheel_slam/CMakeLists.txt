# ====================================================================================================================================
# @file       CMakeLists.txt
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Tuesday, 1st March 2022 4:30:53 pm
# @modified   Wednesday, 25th May 2022 11:16:52 pm
# @project    engineering-thesis
# @brief      CMake file for `velmwheel_slam` package
# 
# 
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Project ============================================================ #

# CMake version
cmake_minimum_required(VERSION 3.8)
# Project header
project(velmwheel_slam)

# ========================================================== Dependencies ========================================================== #

# Ament dependency (always present)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# ===================================================== Source (Python nodes) ====================================================== #

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# ========================================================= Share installs ========================================================= #

# Install shared data
install(
    DIRECTORY
        launch
    DESTINATION
        share/${PROJECT_NAME}
)

# ============================================================== Build ============================================================= #

# Build package using ament
ament_package()
