# ====================================================================================================================================
# @file       slam.launch.py
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Thursday, 7th April 2022 6:50:10 am
# @modified   Wednesday, 25th May 2022 4:44:01 pm
# @project    engineering-thesis
# @brief      Launchfile running middleware of the WUT Velmwheel robot's `slam` module
#    
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Imports ============================================================ #

# Launch imports
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
# Private imports
from launch_common.config import get_config_source
from launch_common.includes import get_launch_source
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
# ============================================================ Constants =========================================================== #

# Name of the component
component_name = 'map_server'

# ===================================================== Components descriptors ===================================================== #


# ================================================================================================================================== #

def generate_launch_description():
    map_path = LaunchConfiguration('map_path')
    map_path_launch_arg = DeclareLaunchArgument("map_path", default_value=get_package_share_directory('velmwheel_gazebo') + '/gazebo/media/maps/elektron_world.yaml')
    return LaunchDescription(
        [
            map_path_launch_arg,

            # Includes
            # IncludeLaunchDescription( **component_description )
            Node(
                package='nav2_map_server',
                namespace='velmwheel',
                executable=component_name,
                name='map_server',
                parameters=[
                    {'yaml_filename': map_path}
                ]

            )
        ]
    )
