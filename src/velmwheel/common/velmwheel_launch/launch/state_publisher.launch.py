# ====================================================================================================================================
# @file       robot_state_publisher.launch.py
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Friday, 4th March 2022 5:57:12 pm
# @modified   Thursday, 26th May 2022 3:33:13 am
# @project    engineering-thesis
# @brief      Launchfile runnign robot_state_publisher and joint_state_publisher configured for Gazebo simulation
#    
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Imports ============================================================ #

# ROS imports
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, ExecuteProcess
from launch_ros.descriptions import ParameterValue
from launch.substitutions import Command, FindExecutable
from launch.event_handlers import OnProcessStart
# Ament imports
from ament_index_python.packages import get_package_share_path
# Private imports
from velmwheel_model.params import ROBOT_NAME
from launch_common.arguments import declare_launch_argument

# =========================================================== Arguments ============================================================ #

# Launch argument: log level of all nodes
declare_velmwheel_log_level_description, velmwheel_log_level_config = declare_launch_argument({
    'name':          'velmwheel_log_level',
    'default_value': 'warn',
    'description':   'Log level of the run nodes'
})

# Launch argument: whether to run state publishers with simulated time ON
declare_use_sim_time_config_description, use_sim_time_config = declare_launch_argument({
    'name':          'use_sim_time',
    'default_value': 'false',
    'description':   'Set to "true" to use simulated time'
})

# Launch argument: whether to run `gzclient` along with the `gzservser`
declare_urdf_path_config_description, urdf_path_config = declare_launch_argument({
    'name':          'urdf_path',
    'default_value': str(get_package_share_path('velmwheel_model') / 'urdf' / 'velmwheel.urdf.xacro'),
    'description':   'Path to the URDF xacro file describing the robot'
})

# ============================================================== Nodes ============================================================= #

# Joint state publisher
joint_state_publisher_description = {

    # Node executable
    'package': 'joint_state_publisher',
    'executable': 'joint_state_publisher',
    'namespace': f'/{ROBOT_NAME}',
    # Output config
    'output': 'both',
    'emulate_tty': True,

    # ROS parameters
    'parameters': [
        { 'source_list': [ f'/{ROBOT_NAME}/base/joint_states' ] },
        { 'use_sim_time': use_sim_time_config                   }
    ],

    # Node's log level
    'arguments': [ '--ros-args', '--log-level', velmwheel_log_level_config ],
}

# Robot state publisher
robot_state_publisher_description = {

    # Node executable
    'package': 'robot_state_publisher',
    'executable': 'robot_state_publisher',
    'namespace': f'/{ROBOT_NAME}',
    # Output configruation
    'output': 'both',

    # ROS parameters
    'parameters': [
        { 'robot_description': ParameterValue( Command(['xacro ', urdf_path_config]), value_type=str ) },
        { 'use_sim_time': use_sim_time_config                                                          }
    ],
    
    # Node's log level
    'arguments': [ '--ros-args', '--log-level', velmwheel_log_level_config ],
}

# =========================================================== Description ========================================================== #

# Description of the launch
launch_description = [
    
    # Arguments
    DeclareLaunchArgument(**declare_velmwheel_log_level_description),
    DeclareLaunchArgument(**declare_use_sim_time_config_description),
    DeclareLaunchArgument(**declare_urdf_path_config_description),

    # Nodes
    Node(**joint_state_publisher_description),
    Node(**robot_state_publisher_description)
    
]

# ============================================================= Launch ============================================================= #

def generate_launch_description():
    return LaunchDescription(launch_description)
