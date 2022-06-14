# ====================================================================================================================================
# @file       base_controller.launch.py
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Friday, 4th March 2022 5:57:12 pm
# @modified   Thursday, 26th May 2022 2:27:55 am
# @project    engineering-thesis
# @brief      Launchfile starting producer's drivers of SICK LIDAR sensors preconfigured for WUT Velmwheel robot
#    
#    
# @note Launchfile has been based on the <sick_scan>/launch/sick_lms_1xx.launch launch file
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Imports ============================================================ #

# ROS includes
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PythonExpression
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
# Private includes
from velmwheel_model.params import ROBOT_NAME
from launch_common.arguments import declare_launch_argument
from launch_common.config import get_config_source

# ======================================================== Launch arguments ======================================================== #

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

# Launch argument: flag stating whether driver node of the left LIDAR sensor should be run 
declare_should_left_run_config_description, should_left_run_config = declare_launch_argument({
    'name':          'with_left_lidar_driver',
    'default_value': 'true',
    'description':   f'If set to "true", the lidar_l node will be run'
})

# Launch argument: flag stating whether driver node of the right LIDAR sensor should be run 
declare_should_right_run_config_description, should_right_run_config = declare_launch_argument({
    'name':          'with_right_lidar_driver',
    'default_value': 'true',
    'description':   f'If set to "true", the lidar_r node will be run'
})

# Launch argument: configruation of the left LIDAR sensor's driver node
declare_left_config_config_description, left_config_config = declare_launch_argument({
    'name':          'left_lidar_driver_config',
    'default_value': get_config_source('velmwheel_laser_driver', 'left_lidar_driver.yaml'),
    'description':   f'Path to the YAML configuration file for the lidar_l node'
})

# Launch argument: configruation of the right LIDAR sensor's driver node
declare_right_config_config_description, right_config_config = declare_launch_argument({
    'name':          'right_lidar_driver_config',
    'default_value': get_config_source('velmwheel_laser_driver', 'right_lidar_driver.yaml'),
    'description':   f'Path to the YAML configuration file for the lidar_r node'
})

# ============================================================== Nodes ============================================================= #

def lidar_description(name, frame_id, config_file, condition_arg):

    """Produces description of the driver of one of LIDAR drivers"""

    return {

        # Controller node
        'package': 'sick_scan',
        'executable': 'sick_generic_caller',
        'name': name,
        # Output config
        'output': 'both',
        'emulate_tty': True,
        # Robot's namespace node
        'namespace': f'/{ROBOT_NAME}',
        # Node's parameters
        'parameters': [

            # Simulation configuration
            { 'use_sim_time': use_sim_time_config },
            # Driver's preconfigured configuration
            { 'scanner_type': 'sick_lms_1xx'      },
            { 'port': '2112'                      },
            { 'use_binary_protocol': True         },
            { 'intensity_resolution_16bit': False },
            { 'min_ang': -2.35619                 },
            { 'max_ang': 2.35619                  },
            { 'range_max': 25.0                   },
            { 'timelimit': 5                      },
            { 'intensity': True                   },
            { 'frame_id': frame_id                },
            # Configuration parameters
            config_file,
            
        ],

        # Remap I/O topics
        'remappings': [

            # Remap output topic
            ( 'cloud', f'/{ROBOT_NAME}/lidars/cloud' ),

        ],
        # Node's log level
        'arguments': [ '--ros-args', '--log-level', velmwheel_log_level_config ],
        # Running condition
        'condition' : IfCondition(PythonExpression(["'", condition_arg, "' == 'true'"]))

    }

# Nodes' descriptions
left_driver_node_description  = lidar_description( 'lidar_l', 'lidar_l_core', left_config_config,  should_left_run_config  )
right_driver_node_description = lidar_description( 'lidar_r', 'lidar_r_core', right_config_config, should_right_run_config )

# ======================================================= Launch description ======================================================= #

launch_description = [
    
    # Arguments
    DeclareLaunchArgument(**declare_velmwheel_log_level_description),
    DeclareLaunchArgument(**declare_use_sim_time_config_description),
    DeclareLaunchArgument(**declare_should_left_run_config_description),
    DeclareLaunchArgument(**declare_should_right_run_config_description),
    DeclareLaunchArgument(**declare_left_config_config_description),
    DeclareLaunchArgument(**declare_right_config_config_description),
    # Nodes
    Node(**left_driver_node_description),
    Node(**right_driver_node_description),

]

# ================================================================================================================================== #

def generate_launch_description():
    return LaunchDescription(launch_description)