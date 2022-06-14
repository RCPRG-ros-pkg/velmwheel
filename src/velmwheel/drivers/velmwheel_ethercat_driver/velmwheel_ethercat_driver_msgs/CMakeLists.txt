# ====================================================================================================================================
# @file       CMakeLists.txt
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Tuesday, 1st March 2022 4:30:53 pm
# @modified   Friday, 27th May 2022 11:44:08 am
# @project    engineering-thesis
# @brief      Minimal CMake file for the 'velmwheel_ethercat_driver_msgs' package
#    
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Project ============================================================ #

# CMake version
cmake_minimum_required(VERSION 3.8)
# Project header
project(velmwheel_ethercat_driver_msgs)

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

# ============================================================ Messages ============================================================ #

# Add message interfaces to the project
rosidl_generate_interfaces(
    
    # Sources
    ${PROJECT_NAME}

        # Process-related services
        "srv/LockMemory.srv"

        # Bus-related services
        "srv/bus/GetBusState.srv"
        "srv/bus/GetMasterState.srv"
        "srv/bus/SetBusState.srv"
        "srv/bus/SetMasterState.srv"
        "srv/bus/GetTimingInfo.srv"

        # Drivers-related services
        "srv/drivers/LoadDriver.srv"
        "srv/drivers/UnloadDriver.srv"
        "srv/drivers/ListDrivers.srv"

    # Dependencies
    DEPENDENCIES
        builtin_interfaces
)

# Export ament dependencies for generated interfaces
ament_export_dependencies(rosidl_default_runtime)

# ============================================================== Build ============================================================= #

# Build package using ament
ament_package()
