# ====================================================================================================================================
# @file       CMakeLists.txt
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Wednesday, 16th March 2022 4:33:51 pm
# @modified   Friday, 27th May 2022 5:19:06 pm
# @project    engineering-thesis
# @brief      CMake file for `velmwheel_ethercat_driver` package
#    
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Project ============================================================ #

# CMake version
cmake_minimum_required(VERSION 3.8)
# Project header
project(velmwheel_ethercat_driver)

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
    pluginlib
    node_common_cpp
    cifx_ethercat
    velmwheel_ethercat_driver_msgs
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

# ============================================================= Plugin ============================================================= #

# Add target
add_library(ethercat_slave_driver INTERFACE)
# Add include directories to the target
target_include_directories(ethercat_slave_driver
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include/public>
        $<INSTALL_INTERFACE:include>
)
# Link target dependencies
ament_target_dependencies(ethercat_slave_driver
    INTERFACE
        rclcpp
        cifx_ethercat
)

# Install targets into the exporting set
install(TARGETS ethercat_slave_driver EXPORT "export_ethercat_slave_driver"
    ARCHIVE  DESTINATION lib
    LIBRARY  DESTINATION lib
    RUNTIME  DESTINATION bin
)

# Install headers
install(
    DIRECTORY
        include/public/
    DESTINATION
        include
)

# Export headers to ament
ament_export_include_directories(include)
# Export dependencies
ament_export_dependencies(rclcpp)
ament_export_dependencies(cifx_ethercat)
# Export targets
ament_export_targets("export_ethercat_slave_driver")

# ============================================================== Node ============================================================== #

# Set lib target names
STRING(REPLACE "velmwheel_" "" LIB_TARGET ${PROJECT_NAME})
# Set executable target names
set(EXEC_TARGET ${PROJECT_NAME})

# Add node
add_ros_component_node(

    # Library target
    LIBRARY_NAME ${LIB_TARGET}
    # Plugin target
    PLUGIN_NAME "velmwheel::EthercatDriver"
    # Executable target
    EXECUTABLE_NAME ${EXEC_TARGET}

    # Sources
    SOURCES
        src/ethercat_driver_impl.cpp
        src/ethercat_driver.cpp
    # Includes
    OVERRIDE_INCLUDES
        $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include/private>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include/public>

    # Ament dependencies
    AMENT_DEPENDENCIES ${DEPENDENCIES}
    
)

# ===================================================== Source (Python nodes) ====================================================== #

# Install Python executables
install(
    
    # Auxiliary ROS nodes
    PROGRAMS
        
        # Process-related services callers
        ${PROJECT_NAME}/memory_locker.py
        
        # Bus-related services callers
        ${PROJECT_NAME}/bus_state_setter.py
        ${PROJECT_NAME}/bus_state_getter.py
        ${PROJECT_NAME}/master_state_setter.py
        ${PROJECT_NAME}/master_state_getter.py
        ${PROJECT_NAME}/timings_getter.py
        
        # Drivers-related services callers
        ${PROJECT_NAME}/driver_loader.py
        ${PROJECT_NAME}/driver_unloader.py
        ${PROJECT_NAME}/drivers_lister.py
        
    # Destination
    DESTINATION lib/${PROJECT_NAME}
    
)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# ========================================================= Share installs ========================================================= #

# Install shared data
install(
    DIRECTORY
        config
    DESTINATION
        share/${PROJECT_NAME}
)

# ============================================================== Build ============================================================= #

# Build package using ament
ament_package()
