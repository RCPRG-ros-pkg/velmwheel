# ====================================================================================================================================
# @file       set_laser_ip.launch.py
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Friday, 4th March 2022 5:57:12 pm
# @modified   Thursday, 26th May 2022 2:27:25 am
# @project    engineering-thesis
# @brief      Launchfile configuring a new IP address for the SICK LIDAR sensor
#    
#    
# @note Launchfile has been based on the <sick_scan>/launch/sick_new_ip.launch launch file
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Imports ============================================================ #

# ROS includes
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch import LaunchDescription
# Private includes
from velmwheel_model.params import ROBOT_NAME
from launch_common.arguments import declare_launch_argument

# ======================================================== Launch arguments ======================================================== #

# Launch argument: log level of all nodes
declare_velmwheel_log_level_description, velmwheel_log_level_config = declare_launch_argument({
    'name':          'velmwheel_log_level',
    'default_value': 'warn',
    'description':   'Log level of the run nodes'
})

# Launch argument: current IP address of the sensor
declare_current_ip_description, current_ip_config = declare_launch_argument({
    'name':        'current_ip',
    'description': 'Current IP address of the sensor'
})

# Launch argument: new IP address of the sensor
declare_new_ip_description, new_ip_config = declare_launch_argument({
    'name':        'new_ip',
    'description': 'Wew IP address of the sensor'
})

# ============================================================== Nodes ============================================================= #

# Nodes' descriptions
node_description = {

    # Controller node
    'package': 'sick_scan',
    'executable': 'sick_generic_caller',
    'name': 'sick_address_configurator',
    # Output config
    'output': 'screen',
    # Robot's namespace node
    'namespace': f'/{ROBOT_NAME}',
    # Node's parameters
    'parameters': [

        # Driver's preconfigured configuration
        { 'scanner_type': 'sick_lms_1xx'                },
        { 'port': 2112                                  },
        { 'use_binary_protocol': True                   },
        { 'timelimit': 5                                },
        { 'min_intensity': 0.0                          },
        { 'start_services':                  True       },
        { 'message_monitoring_enabled':      True       },
        { 'read_timeout_millisec_default':   5000       },
        { 'read_timeout_millisec_startup':   120000     },
        { 'read_timeout_millisec_kill_node': 150000     },
        { 'client_authorization_pw':         'F4724744' },
        # IP configuration
        { 'hostname':       current_ip_config },
        { 'new_IP_address': new_ip_config     },
        
    ],

    # Node's log level
    'arguments': [ '--ros-args', '--log-level', velmwheel_log_level_config ]

}

# ======================================================= Launch description ======================================================= #

launch_description = [

    # Arguments
    DeclareLaunchArgument(**declare_velmwheel_log_level_description),
    DeclareLaunchArgument(**declare_current_ip_description),
    DeclareLaunchArgument(**declare_new_ip_description),
    # Nodes
    Node(**node_description),

]

# ================================================================================================================================== #

def generate_launch_description():
    LaunchDescription(launch_description)