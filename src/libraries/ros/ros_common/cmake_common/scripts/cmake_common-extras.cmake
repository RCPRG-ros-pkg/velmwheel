# ====================================================================================================================================
# @file       cmake_common-extras.cmake
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Thursday, 12th May 2022 11:27:46 pm
# @modified   Thursday, 9th June 2022 3:39:37 am
# @project    engineering-thesis
# @brief      Package's resource file adding resources imported by downstream packages when the `cmake_common` package is found
#    
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# Find dependencies
find_package(rclcpp QUIET REQUIRED)
find_package(rclcpp_components QUIET REQUIRED)
find_package(rosidl_default_generators QUIET REQUIRED)

# Include library
include("${cmake_common_DIR}/arguments.cmake")
include("${cmake_common_DIR}/packages.cmake")
include("${cmake_common_DIR}/ros/add_ament_library.cmake")
include("${cmake_common_DIR}/ros/add_ros_component_node.cmake")
include("${cmake_common_DIR}/ros/link_self_interface.cmake")
