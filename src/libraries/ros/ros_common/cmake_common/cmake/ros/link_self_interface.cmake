# ====================================================================================================================================
# @file       link_self_interface.cmake
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Thursday, 12th May 2022 11:27:46 pm
# @modified   Tuesday, 24th May 2022 8:53:17 pm
# @project    engineering-thesis
# @brief      Definition of the link_self_interface() macro
#    
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ----------------------------------------------------------------------------------
# @brief Links @p target to the @p interface_target declared in the same CMake 
#    roject in a way that is ROS-distros independant
# ----------------------------------------------------------------------------------
macro(link_self_interface target interface_target)
    
    # Set ROS_DISTRO variable
    if(NOT ROS_DISTRO)
        set(ROS_DISTRO $ENV{ROS_DISTRO})
    endif()

    # Link generated messages
    if(ROS_DISTRO STRLESS "humble")

        rosidl_target_interfaces(${target} ${interface_target} "rosidl_typesupport_cpp")
        
    # Since ROS Humble use two-step self-linkage
    else()
    
        # Create target for internally-generated interfaces
        rosidl_get_typesupport_target(cpp_typesupport_target ${interface_target} "rosidl_typesupport_cpp")
        # Link to the target
        target_link_libraries(${target} ${cpp_typesupport_target})

    endif()

endmacro()
