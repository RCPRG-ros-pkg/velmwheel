# ====================================================================================================================================
# @file     CMakeLists.txt
# @author   Tom Moore
# @date     Tuesday, 29th March 2022 12:03:01 pm
# @modified   Thursday, 16th June 2022 2:18:59 pm
# @project  engineering-thesis
# @brief
#    
#    CMake file for the API-exporting version of the `robot_localization` package
#    
# @details This CMake file has been competely rebuilt to avoid building unneccessary targets (nodex and tests) and provide valid
#    exports for the resulting library ( @modified )
# 
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Project ============================================================ #

# CMake version
cmake_minimum_required(VERSION 3.5)
# Project header
project(robot_localization_api)

# ========================================================== Configuration ========================================================= #

# Add extra warnings
add_compile_options(-Wall -Wextra -Wpedantic)

# Configure C++17 (instead of original C++14) to meet rclcpp requirement for std::variant
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Geographiclib installs FindGeographicLib.cmake to this non-standard location
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")

# Set name of the generated messages library
set(MSG_LIB_NAME ${PROJECT_NAME}_msgs)
# Set name of the library
set(LIB_NAME ${PROJECT_NAME})

# ========================================================== Dependencies ========================================================== #

# List dependencies 
list(APPEND LIB_DEPENDENCIES
    angles
    rclcpp
    diagnostic_msgs
    diagnostic_updater
    geographic_msgs
    geometry_msgs
    message_filters
    nav_msgs
    sensor_msgs
    std_msgs
    std_srvs
    tf2
    tf2_eigen
    tf2_geometry_msgs
    tf2_ros
    Eigen3
    Boost
    yaml_cpp_vendor
)

# Utilities dependencies
find_package(cmake_common REQUIRED)
# Find dependencies
find_packages(

    # Ament dependency (always present)
    ament_cmake
    # Common dependencies
    ${LIB_DEPENDENCIES}
    # Interface dependenies
    rosidl_default_generators
    # Find dependencies
    GeographicLib
    
)

# ============================================================ Messages ============================================================ #

# Generate messages
rosidl_generate_interfaces(${MSG_LIB_NAME}

    # Sources
    "srv/FromLL.srv"
    "srv/GetState.srv"
    "srv/SetDatum.srv"
    "srv/SetPose.srv"
    "srv/ToggleFilterProcessing.srv"
    "srv/ToLL.srv"

    # Dependencies
    DEPENDENCIES
        builtin_interfaces
        geometry_msgs
        geographic_msgs
        diagnostic_msgs

    # Tests
    ADD_LINTER_TESTS
    
)

# ============================================================= Source ============================================================= #

# Provide some system headers
include_directories(SYSTEM ${Eigen_INCLUDE_DIRS})

# Add target
add_ament_library(

    # Library name
    LIBRARY_NAME ${LIB_NAME}

    # Library sources
    SOURCES
        src/ekf.cpp
        src/ukf.cpp
        src/filter_base.cpp
        src/filter_utilities.cpp
        src/navsat_transform.cpp
        src/robot_localization_estimator.cpp
        src/ros_filter.cpp
        src/ros_filter_utilities.cpp
        src/ros_robot_localization_listener.cpp

    # Dependencies
    AMENT_DEPENDENCIES ${LIB_DEPENDENCIES}
    
)

# Link additional dependency
ament_target_dependencies(${LIB_NAME} GeographicLib)
# Link internally-generated interfaces
link_self_interface(${LIB_NAME} ${MSG_LIB_NAME})

# ======================================================== Shared resources ======================================================== #

# Install shared resources
install(
    DIRECTORY
        config
    DESTINATION
        share/${PROJECT_NAME}
)

# ============================================================== Build ============================================================= #

# Export GeographicLib library as it cannot be found by default
ament_export_include_directories(${GeographicLib_INCLUDE_DIRS})
ament_export_libraries(${GeographicLib_LIBRARIES})

# Export runtime dependencies for generated messages
ament_export_dependencies(rosidl_default_runtime)

# Build package using ament
ament_package()
