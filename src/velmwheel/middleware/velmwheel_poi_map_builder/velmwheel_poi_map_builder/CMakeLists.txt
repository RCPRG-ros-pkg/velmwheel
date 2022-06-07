# ====================================================================================================================================
# @file       CMakeLists.txt
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Tuesday, 1st March 2022 4:30:53 pm
# @modified   Thursday, 26th May 2022 2:38:16 am
# @project    engineering-thesis
# @brief      Minimal CMake file for the 'velmwheel_poi_map_builder' package
#    
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Project ============================================================ #

# CMake version
cmake_minimum_required(VERSION 3.8)
# Project header
project(velmwheel_poi_map_builder)

# ========================================================== Configuration ========================================================= #

# Add extra warnings
add_compile_options(-Wall -Wextra -Wpedantic)

# Configure C++17 (instead of original C++14) to meet rclcpp requirement for std::variant
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# ========================================================== Dependencies ========================================================== #

# Librarie's dependencies
list(APPEND LINK_DEPENDENCIES
    
    # Build dependencies (ROS)
    rclcpp
    rclcpp_components
    node_common_cpp
    
    # Build dependencies (ROS interfaces)
    sensor_msgs
    visualization_msgs
    velmwheel_poi_map_builder_msgs
    poi_map_msgs
    
    # Build dependencies (TF)
    tf2
    tf2_ros
    
    # Utilities
    pcl_ros
    pcl_conversions
    poi_map_lib
    
    # Velmwheel
    velmwheel_model
)

# Librarie's dependencies
list(APPEND DEPENDENCIES
    
    # Linkable dependencies
    ${LINK_DEPENDENCIES}
    # Other dependencies
    PCL
    
)

# Common dependencies
find_package(cmake_common REQUIRED)
# Ament dependency (always present)
find_packages(

    # Ament dependencies
    ament_cmake
    ament_cmake_python
    # Find dependencies
    ${DEPENDENCIES}

)

# ======================================================== Source (Builder) ======================================================== #

# Set lib target names
STRING(REPLACE "velmwheel_" "" LIB_TARGET ${PROJECT_NAME})
# Set executable target names
set(EXEC_TARGET ${PROJECT_NAME})

# Add node
add_ros_component_node(

    # Library target
    LIBRARY_NAME ${LIB_TARGET}
    # Plugin target
    PLUGIN_NAME "velmwheel::points_of_interest::MapBuilder"
    # Executable target
    EXECUTABLE_NAME ${EXEC_TARGET}

    # Sources
    SOURCES
        src/poi_map_common.cpp
        src/poi_map_strategies.cpp
        src/poi_map_builder_impl.cpp
        src/poi_map_builder.cpp

    # Ament dependencies
    AMENT_DEPENDENCIES ${LINK_DEPENDENCIES}
    
)

# ========================================================== Python nodes ========================================================== #

# Install Python executables
install(
    PROGRAMS
        ${PROJECT_NAME}/show_map.py
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

# ============================================================== Tests ============================================================= #

# Test builds
if(BUILD_TESTING)

    # Find linter
    find_package(ament_lint_auto REQUIRED)
    # Find test dependencies using info from `package.xml`
    ament_lint_auto_find_test_dependencies()

endif()

# ============================================================== Build ============================================================= #

# Install Python package
ament_python_install_package(${PROJECT_NAME})
# Build package using ament
ament_package()
