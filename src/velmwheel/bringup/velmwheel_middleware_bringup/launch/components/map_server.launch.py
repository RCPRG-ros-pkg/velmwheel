
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
