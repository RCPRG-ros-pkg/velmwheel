# ====================================================================================================================================
# @file       odom_fusion.launch.py
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Friday, 4th March 2022 5:57:12 pm
# @modified   Wednesday, 25th May 2022 4:44:01 pm
# @project    engineering-thesis
# @brief      Launchfile for the odom fusion submodule for the the WUT Velmwheel robot's driveline
#    
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Imports ============================================================ #

# ROS imports
from launch import LaunchDescription
from launch.substitutions import PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
# Private imports
from velmwheel_model.params import ROBOT_NAME
from launch_common.arguments import declare_launch_argument
from launch_common.config import get_config_source

# ============================================================ Arguments =========================================================== #

# Launch argument: log level of all nodes
declare_velmwheel_log_level_description, velmwheel_log_level_config = declare_launch_argument({
    'name':          'velmwheel_log_level',
    'default_value': 'warn',
    'description':   'Log level of the run nodes'
})

# Launch argument: whether to run `gzclient` along with the `gzservser`
declare_use_sim_time_config_description, use_sim_time_config = declare_launch_argument({
    'name':          'use_sim_time',
    'default_value': 'false',
    'description':   'Set to "true" to use simulated time'
})

# Launch argument: flag stating whether node should be run (auxiliary setting for higher launchfiles)
declare_should_run_config_description, should_run_config = declare_launch_argument({
    'name':          'with_odom_fusion',
    'default_value': 'true',
    'description':   f'If set to "true", the odom_fusion node will be run'
})

# Launch argument: configruation of the node
declare_config_config_description, config_config = declare_launch_argument({
    'name':          'odom_fusion_config',
    'default_value': '',
    'description':   f'Path to the YAML configuration file for the odom_fusion node'
})

# Launch argument: whether to run 'custom' or 'original' implementation fo `robot_localization`
declare_type_config_description, type_config = declare_launch_argument({
    'name':          'odom_fusion_type',
    'default_value': 'custom',
    'description':   'Whether to run "custom" or "original" implementation fo `robot_localization`',
    'choices':       [ 'custom', 'original' ]
})

# ======================================================== Helper constants ======================================================== #

# Path to the default configuration of the custom-fusion
custom_cfg_default = get_config_source('velmwheel_odom_fusion', 'odom_fustion_custom.yaml')
# Path to the default configuration of the original-fusion
original_cfg_default = get_config_source('velmwheel_odom_fusion', 'odom_fustion_original.yaml')

# ======================================================= Custom-fusion node ======================================================= #

# Basic (custom) implementation of the robot localization
def custom_fusion_description(condition, config_file):
    return {

        # Controller node
        'package': 'velmwheel_odom_fusion',
        'executable': 'velmwheel_odom_fusion',
        'name': 'odom_fusion',
        # Robot's namespace node
        'namespace': f'/{ROBOT_NAME}',
        # Node's parameters
        'parameters': [

            # Configruation file
            config_file,
            # Simulation configuration
            { 'use_sim_time': use_sim_time_config },

        ],

        # Remap I/O topics
        'remappings': [

            # Inputs
            ( 'velocity_setpoint', f'/{ROBOT_NAME}/base/velocity_setpoint' ),

        ],

        # Node's log level
        'arguments': [ '--ros-args', '--log-level', velmwheel_log_level_config ],

        # Running condition
        'condition' : IfCondition(PythonExpression([

            "'", should_run_config, "' == 'true'",
            " and ",
            "'", type_config, "' == 'custom'",
            " and ",
            *condition
            
        ]))
    }

# Specialization of launch actions
custom_fusion = [
    custom_fusion_description( condition=[ "'", config_config, "' == ''" ], config_file=custom_cfg_default ),
    custom_fusion_description( condition=[ "'", config_config, "' != ''" ], config_file=config_config      )
]

# ====================================================== Original-fusion node ====================================================== #

# Original implementation of the robot localization
def original_fusion_description(condition, config_file):
    return {

        # Controller node
        'package': 'robot_localization',
        'executable': 'ekf_node',
        'name': 'odom_fusion',
        # Robot's namespace node
        'namespace': f'/{ROBOT_NAME}',
        # Node's parameters
        'parameters': [

            # Configruation file
            config_file,
            # Simulation configuration
            { 'use_sim_time': use_sim_time_config },

        ],

        # Remap I/O topics
        'remappings': [

            # Inputs
            ( 'odom/encoders',     f'/{ROBOT_NAME}/odom/encoders'          ),
            ( 'odom/laser',        f'/{ROBOT_NAME}/odom/laser'             ),
            ( 'cmd_vel',           f'/{ROBOT_NAME}/base/velocity_setpoint' ),
            # Outputs
            ( 'odometry/filtered', f'/{ROBOT_NAME}/odom/filtered'          ),

        ],

        # Node's log level
        'arguments': [ '--ros-args', '--log-level', velmwheel_log_level_config ],
        
        # Running condition
        'condition' : IfCondition(PythonExpression([

            "'", should_run_config, "' == 'true'",
            " and ",
            "'", type_config, "' == 'original'",
            " and ",
            *condition
            
        ]))
    }

# Specialization of launch actions
original_fusion = [
    original_fusion_description( condition=[ "'", config_config, "' == ''" ], config_file=original_cfg_default ),
    original_fusion_description( condition=[ "'", config_config, "' != ''" ], config_file=config_config        )
]

# ======================================================= Launch description ======================================================= #

# Description of the launch
launch_description = [
    
    # Arguments
    DeclareLaunchArgument(**declare_velmwheel_log_level_description),
    DeclareLaunchArgument(**declare_use_sim_time_config_description),
    DeclareLaunchArgument(**declare_should_run_config_description),
    DeclareLaunchArgument(**declare_config_config_description),
    DeclareLaunchArgument(**declare_type_config_description),
    
    # Nodes
    *[ Node(**node) for node in custom_fusion   ],
    *[ Node(**node) for node in original_fusion ],
    
]

# ============================================================= Launch ============================================================= #

def generate_launch_description():
    return LaunchDescription(launch_description)

# ================================================================================================================================== #
