# ====================================================================================================================================
# @file       CMakeLists.txt
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Tuesday, 1st March 2022 4:30:53 pm
# @modified   Wednesday, 25th May 2022 11:16:52 pm
# @project    engineering-thesis
# @brief      CMake file for `velmwheel_gazebo` package
# 
# 
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Project ============================================================ #

# CMake version
cmake_minimum_required(VERSION 3.8)
# Project header
project(velmwheel_msgs)

# ========================================================== Dependencies ========================================================== #

# Ament dependency (always present)
find_package(ament_cmake REQUIRED)
# Interfaces dependencies
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# =========================================================== Interfaces =========================================================== #

# Add interfaces to the project
rosidl_generate_interfaces(${PROJECT_NAME}

    # Messages
    "msg/WheelEnum.msg"
    "msg/Wheel.msg"
    "msg/Wheels.msg"
    "msg/Encoder.msg"
    "msg/EncoderStamped.msg"
    "msg/EncoderRaw.msg"
    "msg/Encoders.msg"
    "msg/EncodersStamped.msg"

    # Dependencies
    DEPENDENCIES std_msgs

)

# Export ament dependencies
ament_export_dependencies(rosidl_default_runtime)

# ============================================================== Build ============================================================= #

# Build package using ament
ament_package()
