
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
component_name = 'planner_server'

# ===================================================== Components descriptors ===================================================== #


# ================================================================================================================================== #

def generate_launch_description():

    return LaunchDescription(
        [
            # Includes
            # IncludeLaunchDescription( **component_description )
            Node(
                package='nav2_planner',
                namespace='velmwheel',
                executable=component_name,
                name='global_planner',
                parameters=[
                    get_package_share_directory('velmwheel_middleware_bringup')+'/config/global_planner.yaml'
                ]

            )
        ]
    )
