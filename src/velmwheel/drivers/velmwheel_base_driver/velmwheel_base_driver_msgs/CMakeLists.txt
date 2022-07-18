# ====================================================================================================================================
# @file       CMakeLists.txt
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Monday, 13th June 2022 8:33:54 pm
# @modified   Friday, 1st July 2022 5:21:58 pm
# @project    engineering-thesis
# @brief      Minimal CMake file for the 'velmwheel_base_driver_msgs' package
#    
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Project ============================================================ #

# CMake version
cmake_minimum_required(VERSION 3.8)
# Project header
project(velmwheel_base_driver_msgs)

# ========================================================== Configuration ========================================================= #

# Add extra warnings
add_compile_options(-Wall -Wextra -Wpedantic)

# Configure C++17 (instead of original C++14) to meet rclcpp requirement for std::variant
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# ========================================================== Dependencies ========================================================== #

# Ament dependency (standard buildtool dependencies)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# Interfaces dependencies
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

# ============================================================ Messages ============================================================ #

# Add message interfaces to the project
rosidl_generate_interfaces(
    
    # Sources
    ${PROJECT_NAME}

        # Messages
        "msg/WheelsStatus.msg"
        # Services
        "srv/Enable.srv"
        "srv/GetState.srv"
        "srv/ResetFailure.srv"

    # Dependencies
    DEPENDENCIES
        builtin_interfaces
        std_msgs
)

# Export ament dependencies for generated interfaces
ament_export_dependencies(rosidl_default_runtime)

# ============================================================== Build ============================================================= #

# Build package using ament
ament_package()
