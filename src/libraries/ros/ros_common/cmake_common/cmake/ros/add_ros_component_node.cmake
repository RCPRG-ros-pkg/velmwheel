# ====================================================================================================================================
# @file       add_ros_component_node.cmake
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Thursday, 12th May 2022 11:27:46 pm
# @modified   Friday, 27th May 2022 5:45:19 pm
# @project    engineering-thesis
# @brief      Definition of the add_ros_component_node() function
#    
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ----------------------------------------------------------------------------------
# @brief Calls standard ROS2 CMake routines to declare ROS node component library
#    and registers it as an ament plugin.
#
# @param LIBRARY_NAME [NAME]
#    name of the library target
# @param PLUGIN_NAME [NAME]
#    fully qualified name of the node plugin class
# @param EXECUTABLE_NAME [NAME]
#    name of the node executable
# @param SOURCES [NAMES...]
#    list of source files
# @param ADDITIONAL_INCLUDES [NAMES...] (optional)
#    list of additional include interfaces
# @param OVERRIDE_INCLUDES [NAMES...] (optional)
#    list of include interfaces to be registered instead of default ones
# @param AMENT_DEPENDENCIES [NAMES...] (optional)
#    list of dependencies to be passed to `ament_target_dependencies()`
# ----------------------------------------------------------------------------------
function(add_ros_component_node)
    
    # -------------------------- Parse arguments -------------------------

    # Single-value arguments
    set(SINGLE_ARGUMENTS
        LIBRARY_NAME
        PLUGIN_NAME
        EXECUTABLE_NAME
    )

    # Multi-value arguments
    set(MULTI_ARGUMENTS
        SOURCES
        ADDITIONAL_INCLUDES
        OVERRIDE_INCLUDES
        AMENT_DEPENDENCIES
    )

    # Set arg prefix
    set(ARG_PREFIX "ARG")
    # Parse arguments
    cmake_parse_arguments(${ARG_PREFIX}
        ""
        "${SINGLE_ARGUMENTS}"
        "${MULTI_ARGUMENTS}"
        ${ARGN}
    )
    
    # ------------------------- Validate arguments -----------------------

    # Check if target name has been given
    if(NOT ARG_LIBRARY_NAME)
    message(FATAL_ERROR "add_ros_component_node() must be invoked with shared library name")
    endif()
    
    # Check if plugin name has been given
    if(NOT ARG_PLUGIN_NAME)
    message(FATAL_ERROR "add_ros_component_node() must be invoked with plugin name")
    endif()
    
    # Check if target executable name has been given
    if(NOT ARG_EXECUTABLE_NAME)
    message(FATAL_ERROR "add_ros_component_node() must be invoked with target executable name")
    endif()
    
    # Check if source files has been given
    if(NOT ARG_SOURCES)
        message(FATAL_ERROR "add_ros_component_node() must be invoked with at least one source file")
    endif()

    # --------------------------- Define targets -------------------------

    # Add target
    add_library(${ARG_LIBRARY_NAME} SHARED ${ARG_SOURCES})
    # Add include directories to the target
    if(NOT ARG_OVERRIDE_INCLUDES)
        target_include_directories(${ARG_LIBRARY_NAME}
            PRIVATE
                "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                "$<INSTALL_INTERFACE:include>"
                ${ARG_ADDITIONAL_INCLUDES}
        )
    else()
        target_include_directories(${ARG_LIBRARY_NAME} PRIVATE ${ARG_OVERRIDE_INCLUDES})
    endif()
    # Link target dependencies
    if(ARG_AMENT_DEPENDENCIES)
        ament_target_dependencies(${ARG_LIBRARY_NAME} ${ARG_AMENT_DEPENDENCIES})
    endif()
    # Register shared node component
    rclcpp_components_register_node(${ARG_LIBRARY_NAME}
        PLUGIN
            ${ARG_PLUGIN_NAME}
        EXECUTABLE
            ${ARG_EXECUTABLE_NAME}
    )
    
    # Add visibility definition as the node is build as a shared library (specific for Windows-compatibility)
    target_compile_definitions(${ARG_LIBRARY_NAME}
        PRIVATE
            "RCLCPP_BUILDING_LIBRARY"
    )

    # Install targets
    install(TARGETS ${ARG_LIBRARY_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
    )

endfunction()
