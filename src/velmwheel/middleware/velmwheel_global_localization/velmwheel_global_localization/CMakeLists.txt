# ====================================================================================================================================
# @file       CMakeLists.txt
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Wednesday, 16th March 2022 4:33:51 pm
# @modified   Thursday, 26th May 2022 2:37:16 am
# @project    engineering-thesis
# @brief      CMake file for `velmwheel_global_localization` package
#    
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Project ============================================================ #

# CMake version
cmake_minimum_required(VERSION 3.8)
# Project header
project(velmwheel_global_localization)

# ========================================================== Configuration ========================================================= #

# Add extra warnings
add_compile_options(-Wall -Wextra -Wpedantic)

# Require C++ 17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# ========================================================== Dependencies ========================================================== #

# Librarie's dependencies
list(APPEND DEPENDENCIES
    
    # Build dependencies (ROS)
    rclcpp
    rclcpp_components
    node_common_cpp
    
    # Build dependencies (ROS interfaces)
    geometry_msgs
    sensor_msgs
    nav_msgs
    visualization_msgs
    poi_map_msgs
    
    # Build dependencies (TF)
    tf2
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
    
    # Utilities
    pcl_ros
    pcl_conversions
    poi_map_lib
    
    # Velmwheel
    velmwheel_global_localization_msgs
    velmwheel_common
    
    # Libraries
    Eigen3
    PCL
    
)

# Common dependencies
find_package(cmake_common REQUIRED)
# Ament dependency (always present)
find_packages(

    # Ament dependencies
    ament_cmake
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
    PLUGIN_NAME "velmwheel::GlobalLocalization"
    # Executable target
    EXECUTABLE_NAME ${EXEC_TARGET}

    # Sources
    SOURCES
        src/global_localization.cpp
        src/global_localization_impl.cpp

    # Ament dependencies
    AMENT_DEPENDENCIES ${DEPENDENCIES}
    
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

# ============================================================== Tests ============================================================= #

# Test builds
if(BUILD_TESTING)

    # Find linter
    find_package(ament_lint_auto REQUIRED)
    # Find test dependencies using info from `package.xml`
    ament_lint_auto_find_test_dependencies()

endif()

# ============================================================== Build ============================================================= #

# Build package using ament
ament_package()
