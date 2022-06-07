# ====================================================================================================================================
# @file       CMakeLists.txt
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Wednesday, 16th March 2022 4:33:51 pm
# @modified   Friday, 27th May 2022 5:13:15 pm
# @project    engineering-thesis
# @brief      CMake file for `velmwheel_bias_estimator` package
#    
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Project ============================================================ #

# CMake version
cmake_minimum_required(VERSION 3.8)
# Project header
project(velmwheel_bias_estimator)

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
    std_msgs
    std_srvs
    geometry_msgs
    sensor_msgs
    tf2
    tf2_eigen
    tf2_geometry_msgs
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
    PLUGIN_NAME "velmwheel::BiasEstimator"
    # Executable target
    EXECUTABLE_NAME ${EXEC_TARGET}

    # Sources
    SOURCES
        src/bias_estimator.cpp

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
