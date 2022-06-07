# ====================================================================================================================================
# @file       CMakeLists.txt
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Tuesday, 1st March 2022 4:30:53 pm
# @modified   Wednesday, 25th May 2022 4:44:01 pm
# @project    engineering-thesis
# @brief      Minimal CMake file for the 'velmwheel_poi_map_builder_msgs' package
#    
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Project ============================================================ #

# CMake version
cmake_minimum_required(VERSION 3.8)
# Project header
project(velmwheel_poi_map_builder_msgs)

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

# ============================================================ Messages ============================================================ #

# Add message interfaces to the project
rosidl_generate_interfaces(
    
    # Sources
    ${PROJECT_NAME}
        "srv/ChangeMode.srv"

)

# Export ament dependencies for generated interfaces
ament_export_dependencies(rosidl_default_runtime)

# ============================================================== Build ============================================================= #

# Build package using ament
ament_package()
