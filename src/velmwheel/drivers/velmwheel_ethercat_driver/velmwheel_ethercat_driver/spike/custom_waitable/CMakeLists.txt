# ====================================================================================================================================
# @file       CMakeLists.txt
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Wednesday, 16th March 2022 4:33:51 pm
# @modified   Thursday, 16th June 2022 3:24:29 pm
# @project    engineering-thesis
# @brief      CMake file for `velmwheel_ethercat_driver` package
#    
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Project ============================================================ #

# CMake version
cmake_minimum_required(VERSION 3.8)
# Project header
project(custom_waitable_spike)

# ========================================================== Configuration ========================================================= #

# Add extra warnings
add_compile_options(-Wall -Wextra -Wpedantic)

# Require C++ 17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# ========================================================== Dependencies ========================================================== #

# Common dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# ============================================================= Plugin ============================================================= #

# Add target
add_executable(${PROJECT_NAME} custom_waitable)
# Link rclcpp
ament_target_dependencies(${PROJECT_NAME} rclcpp)

# ============================================================== Build ============================================================= #

# Build package using ament
ament_package()
