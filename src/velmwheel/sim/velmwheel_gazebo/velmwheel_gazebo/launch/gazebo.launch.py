# ====================================================================================================================================
# @file       robot_spawner.launch.py
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Friday, 4th March 2022 5:57:12 pm
# @modified   Wednesday, 25th May 2022 11:54:10 pm
# @project    engineering-thesis
# @brief      Launchfile running Gazebo simulation
# 
# 
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Imports ============================================================ #

# System imports
from os import path
# ROS imports
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
# Ament imports
from ament_index_python.packages import get_package_share_directory
# Private imports
from launch_common.arguments import declare_launch_argument
from launch_common.includes import get_launch_source

# =========================================================== Arguments ============================================================ #

# Launch argument: whether to run `gzclient` along with the `gzservser`
declare_with_gui_config_description, with_gui_config = declare_launch_argument({
    'name':          'with_gui',
    'default_value': 'false',
    'description':   'Set to "false" to run headless'
})

# Launch argument: whether the launch needs to shutdown when the gzserver exits
declare_gzserver_required_config_description, gzserver_required_config = declare_launch_argument({
    'name':          'gzserver_required',
    'default_value': 'true',
    'description':   'Sets whether the launch needs to shutdown when the gzserver exits'
})

# Launch argument: whether the launch needs to shutdown when the gzclient exits
declare_gzclient_required_config_description, gzclient_required_config = declare_launch_argument({
    'name':          'gzclient_required',
    'default_value': 'true',
    'description':   'Sets whether the launch needs to shutdown when the gzclient exits'
})

# Launch argument: name of the world to be run
declare_sim_world_config_description, sim_world_config = declare_launch_argument({
    'name':          'sim_world',
    'default_value': '',
    'description':   'Name of the world to be run (either relative to \'${prefix}/gazebo/media/world\' directory or absolute)'
})

# ============================================================= Helpers ============================================================ #

def get_world_path():

    """
    Returns
    -------
    `launch.substitutions.PythonExpression` representing path to the world to be run
    """

    return PythonExpression([ 
        "'", sim_world_config, "'", "if len('", sim_world_config, "') == 0 or '", sim_world_config, "'.startswith('/')",
        "else",
        "'worlds/", sim_world_config,"'"
    ])


# ============================================================== Nodes ============================================================= #

# Gazebo simulation runner
gazebo_server = {

    # Gazebo run launch
    'launch_description_source': get_launch_source('gazebo_ros', 'gzserver.launch.py'),

    # Launch Parameters
    'launch_arguments': {
        'server_required': gzserver_required_config,
        'verbose': 'true',
        'world': get_world_path()
    }.items()
}

# Gazebo simulation viewer
gazebo_client = {

    # Gazebo run launch
    'launch_description_source': get_launch_source('gazebo_ros', 'gzclient.launch.py'),

    # Launch Parameters
    'launch_arguments': {
        'gui_required': gzclient_required_config,
        'gui_required': 'true'
    }.items(),

    # Run condition
    'condition': IfCondition(with_gui_config)
}

# =========================================================== Description ========================================================== #

# Description of the launch
launch_description = [
    
    # Arguments
    DeclareLaunchArgument(**declare_with_gui_config_description),
    DeclareLaunchArgument(**declare_gzserver_required_config_description),
    DeclareLaunchArgument(**declare_gzclient_required_config_description),
    DeclareLaunchArgument(**declare_sim_world_config_description),

    # Nodes
    IncludeLaunchDescription(**gazebo_server),
    IncludeLaunchDescription(**gazebo_client),
    
]

# ============================================================= Launch ============================================================= #

def generate_launch_description():
    return LaunchDescription(launch_description)
