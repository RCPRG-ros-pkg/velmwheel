# ====================================================================================================================================
# @file       robot_at.launch.py
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Friday, 4th March 2022 5:57:12 pm
# @modified   Friday, 1st July 2022 9:51:15 pm
# @project    engineering-thesis
# @brief      Helper launchfile runnig TF static transform publishers setting frame identified with the robot's psoe in the world
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
declare_robot_at_description, robot_at = declare_launch_argument({
    'name':          'robot_at',
    'default_value': '',
    'description':   f'Name of the frame at which \'{ROBOT_NAME}\' should be placed',
    'choices':       [ '', 'odom_pose', 'laser_odom_pose', 'odom_filtered_pose', 'slam_pose', f'sim_{ROBOT_NAME}' ]
})

# ============================================================== Nodes ============================================================= #

# Node publishing <frame> ---> /odom identitytransform
odom_transform_publisher_description = {

    # Node executable
    'package': 'tf2_ros',
    'executable': 'static_transform_publisher',
    'name': 'robot_transform_publisher',
    # Namespace
    'namespace': f'/{ROBOT_NAME}',
    # Arguments
    'arguments': [
        '--x',     '0',
        '--y',     '0',
        '--z',     '0',
        '--roll',  '0',
        '--pitch', '0',
        '--yaw',   '0',
        '--frame-id', robot_at,
        '--child-frame-id', f'{ROBOT_NAME}'
    ],

    # Run condition
    'condition': IfCondition( PythonExpression(['"', robot_at, '" != ""']) )
}

# =========================================================== Description ========================================================== #

# Description of the launch
launch_description = [
    
    # Arguments
    DeclareLaunchArgument(**declare_robot_at_description),
    # Nodes
    Node(**odom_transform_publisher_description),
    
]

# ============================================================= Launch ============================================================= #

def generate_launch_description():
    return LaunchDescription(launch_description)
