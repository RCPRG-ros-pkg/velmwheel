# ====================================================================================================================================
# @file       rviz.launch.py
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Friday, 4th March 2022 5:57:12 pm
# @modified   Wednesday, 25th May 2022 11:16:52 pm
# @project    engineering-thesis
# @brief      Launchfile running RVIZ configured for the WUT Velmwheel robot
# 
# 
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Imports ============================================================ #

# ROS imports
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, Shutdown, TimerAction, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
# Private imports
from launch_common.config import get_config_source
from launch_common.arguments import declare_launch_argument

# =========================================================== Arguments ============================================================ #

# Launch argument: log level of all nodes
declare_velmwheel_log_level_description, velmwheel_log_level_config = declare_launch_argument({
    'name':          'velmwheel_log_level',
    'default_value': 'warn',
    'description':   'Log level of the run nodes'
})

# Launch argument: whether to run `gzclient` along with the `gzservser`
declare_with_rviz_config_description, with_rviz_config = declare_launch_argument({
    'name':          'with_rviz',
    'default_value': 'false',
    'description':   'Set to "false" to run without rviz'
})

# Launch argument: whether the launch needs to shutdown when the rviz exits
declare_rviz_required_config_description, rviz_required_config = declare_launch_argument({
    'name':          'rviz_required',
    'default_value': 'true',
    'description':   'Sets whether the launch needs to shutdown when the rviz exits'
})

# Launch argument: time after which RIVZ will be run
declare_delay_config_description, delay_config = declare_launch_argument({
    'name':          'rviz_delay',
    'default_value':  '2.0',
    'description':   'Time after which RIVZ will be run in [s]'
})

# Launch argument: RVIZ configuration file
declare_rviz_config_config_description, rviz_config_config = declare_launch_argument({
    'name':          'rviz_config',
    'default_value': '',
    'description':   'RVIZ configuration file'
})

# ======================================================= Nodes descriptions ======================================================= #

# Description of the `state_publisher` node
def rviz_description(condition, extra_config={}):
    return {

        # Node executable
        'package': 'rviz2',
        'executable': 'rviz2',
        'name': 'rviz2',
        # Output config
        'output': 'both',
        'emulate_tty': True,
        # Node's arguments
        'arguments': [ 

            # Configuration file
            "-d", PythonExpression(["'", rviz_config_config, "' if '", rviz_config_config, "' != '' else ''" ]),
            # Node's log level
            '--ros-args', '--log-level', velmwheel_log_level_config,
            
        ],
        
        # Run condition
        'condition': IfCondition(PythonExpression([ "'", with_rviz_config, "' == 'true' ", " and ", *condition ])),

        # Additional config
        **extra_config
        
    }

# ======================================================== Nodes descriptors ======================================================= #

rviz_descriptor = [
    rviz_description( condition=[ "'", rviz_required_config, "' != 'true' " ]                                         ),
    rviz_description( condition=[ "'", rviz_required_config, "' == 'true' " ], extra_config={ 'on_exit': Shutdown() } )
]

# =========================================================== Description ========================================================== #

# Description of the launch
launch_description = [
    
    # Arguments
    DeclareLaunchArgument(**declare_velmwheel_log_level_description),
    DeclareLaunchArgument(**declare_with_rviz_config_description),
    DeclareLaunchArgument(**declare_rviz_required_config_description),
    DeclareLaunchArgument(**declare_delay_config_description),
    DeclareLaunchArgument(**declare_rviz_config_config_description),

    # Nodes
    *[ TimerAction( period=delay_config, actions=[ Node(**rviz_node) ] ) for rviz_node in rviz_descriptor ]
    
]

# ============================================================= Launch ============================================================= #

def generate_launch_description():
    return LaunchDescription(launch_description)
