# ====================================================================================================================================
# @file       CMakeLists.txt
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Wednesday, 16th March 2022 4:33:51 pm
# @modified   Wednesday, 25th May 2022 4:44:01 pm
# @project    engineering-thesis
# @brief      CMake file for `velmwheel_base_controller` package
#    
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Project ============================================================ #

# CMake version
cmake_minimum_required(VERSION 3.8)
# Project header
project(velmwheel_base_controller)

# ========================================================== Configuration ========================================================= #

# Add extra warnings
add_compile_options(-Wall -Wextra -Wpedantic)

# Require C++ 17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# ========================================================== Dependencies ========================================================== #

# Librarie's dependencies
list(APPEND DEPENDENCIES
    rclcpp
    rclcpp_components
    node_common_cpp
    geometry_msgs
    nav_msgs
    tf2_ros
    tf2_geometry_msgs
    velmwheel_model
    velmwheel_msgs
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
    PLUGIN_NAME "velmwheel::BaseController"
    # Executable target
    EXECUTABLE_NAME ${EXEC_TARGET}

    # Sources
    SOURCES
        src/base_controller.cpp

    # Ament dependencies
    AMENT_DEPENDENCIES ${DEPENDENCIES}
    
)

# ========================================================= Share installs ========================================================= #

# Install shared data
install(
    DIRECTORY
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