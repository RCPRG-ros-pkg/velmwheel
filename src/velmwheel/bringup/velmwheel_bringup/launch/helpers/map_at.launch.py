# ====================================================================================================================================
# @file       map_at.launch.py
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Friday, 4th March 2022 5:57:12 pm
# @modified   Wednesday, 25th May 2022 11:12:58 pm
# @project    engineering-thesis
# @brief      Helper launchfile runnig TF static transform publishers setting frame identified with the 'map' frame in the world
#    
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Imports ============================================================ #

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from launch_common.arguments import declare_launch_argument
from velmwheel_model.params import ROBOT_NAME

# =========================================================== Arguments ============================================================ #

# Launch argument: name of the frame to place /odom at
declare_map_at_description, map_at = declare_launch_argument({
    'name':          'map_at',
    'default_value': '',
    'description':   'Name of the frame at which \'odom\' should be placed',
    'choices':       [ '', 'world' ]
})

# ============================================================== Nodes ============================================================= #

# Node publishing <frame> ---> /odom identitytransform
odom_transform_publisher_description = {

    # Node executable
    'package': 'tf2_ros',
    'executable': 'static_transform_publisher',
    'name': 'map_transform_publisher',
    # Namespace
    'namespace': f'/{ROBOT_NAME}',
    # Arguments
    'arguments': ['0', '0', '0', '0', '0', '0', map_at, 'map'],

    # Run condition
    'condition': IfCondition( PythonExpression(['"', map_at, '" != ""']) )
}

# =========================================================== Description ========================================================== #

# Description of the launch
launch_description = [
    
    # Arguments
    DeclareLaunchArgument(**declare_map_at_description),
    # Nodes
    Node(**odom_transform_publisher_description),
    
]

# ============================================================= Launch ============================================================= #

def generate_launch_description():
    return LaunchDescription(launch_description)
