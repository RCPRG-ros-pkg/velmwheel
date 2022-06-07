# ====================================================================================================================================
# @file       CMakeLists.txt
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Tuesday, 1st March 2022 4:30:53 pm
# @modified   Wednesday, 25th May 2022 4:43:52 pm
# @project    engineering-thesis
# @brief      Minimal CMake file for the 'poi_map_msgs' package
#    
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Project ============================================================ #

# CMake version
cmake_minimum_required(VERSION 3.8)
# Project header
project(poi_map_msgs)

# ========================================================== Configuration ========================================================= #

# Add extra warnings
add_compile_options(-Wall -Wextra -Wpedantic)

# Configure C++17 (instead of original C++14) to meet rclcpp requirement for std::variant
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Options
option(BUILD_DOC "If TRUE documentation will be built" ON)

# ========================================================== Dependencies ========================================================== #

# Ament dependency (standard buildtool dependencies)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# Build dependencies (ROS interfaces)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# ============================================================ Messages ============================================================ #

# Add message interfaces to the project
rosidl_generate_interfaces(
    
    # Sources
    ${PROJECT_NAME}
        "msg/Point2D.msg"
        "msg/Marker.msg"
        "msg/MarkersStamped.msg"
        "msg/MarkersMap.msg"
        "srv/SaveMap.srv"
        "srv/LoadMap.srv"

    # Dependencies
    DEPENDENCIES
        std_msgs
        geometry_msgs

)

# Export ament dependencies for generated interfaces
ament_export_dependencies(rosidl_default_runtime)

# =============================================================== Docs ============================================================= #

# Build doc
if(BUILD_DOC)
    add_subdirectory(doc)
endif()

# ============================================================== Build ============================================================= #

# Build package using ament
ament_package()
