# ====================================================================================================================================
# @file       node.py
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Thursday, 7th April 2022 5:18:37 am
# @modified   Friday, 15th July 2022 5:22:33 pm
# @project    engineering-thesis
# @brief      Set of helper functions utilized by launchfiles across velmwheel_slam package
#    
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Imports ============================================================ #

# Ament imports
from ament_index_python.packages import get_package_share_path
# Launch imports
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PythonExpression
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
# Private imports
from launch_common.arguments import declare_launch_argument
from velmwheel_launch.node import generate_component_launch_descriptor
from velmwheel_model.params import ROBOT_NAME

# =========================================================== Parameters =========================================================== #

# Launch argument: scans source for the SLAM toolbox
declare_slam_source_scan_config_description, slam_source_scan_config = declare_launch_argument({
    'name':          'slam_source_scan',
    'default_value': 'combined',
    'description':   'Enumeration pointing out what soruce of laser scans should be used by the SLAM toolbox. ' +
                        'Possible values are: '                                                                 + '\n' +
                        '  - left: raw left laser scan from \'lidar_l/scan\' topic'                             + '\n' +
                        '  - right: raw right laser scan from \'lidar_r/scan\' topic'                           + '\n' +
                        '  - both: raw laser scans from \'lidars/scan\' topic (this use cas is buggish, as '    +
                        '    current implementation of slam_toolbox fails to properly handle scans from '       +
                        '    multiple sensors'                                                                  + '\n' +
                        '  - combined: laser scans from \'lidars/cloud/combined\' topic converted into the '    +
                        '    form of laser scans using pointcloud-to-laserscan node; conversion results are '   + 
                        '    published at \'lidars/scan/combined\' topic',
    'choices': [ 'left', 'right', 'both', 'combined' ]
})

# ======================================================== Auxiliary launch ======================================================== #

def generate_scans_converter_descriptor():

    '''Generates launch description of the pointcloud_to_laserscan conversion node for SLAM toolbox input
    
    Parameters
    ----------
    condition : Any
        roslaunch condition for running the node
    '''
    return {

        # Controller node
        'package': 'pointcloud_to_laserscan',
        'executable': 'pointcloud_to_laserscan_node',
        'name': 'pointcloud_to_laserscan',
        # Robot's namespace node
        'namespace': f'/{ROBOT_NAME}',
        # Output config
        'output': 'both',
        'emulate_tty': True,
        # Node's parameters
        'parameters': [

            # Simulation configuration
            { 'use_sim_time': LaunchConfiguration('use_sim_time') },
            # Other parameters (@todo Make this magic numbers more verbous)
            { 'angle_min'       : -2.356195  },
            { 'angle_max'       :  2.356195  },
            { 'range_min'       :  0.06      },
            { 'range_max'       :  20.0      },
            { 'angle_increment' :  0.0087266 }

        ],

        # Remap I/O topics
        'remappings': [
            ( 'cloud_in', f'/{ROBOT_NAME}/lidars/cloud/combined' ),
            ( 'scan',     f'/{ROBOT_NAME}/lidars/scan/combined'  ),
        ],
        
        # Node's log level
        'arguments': [ '--ros-args', '--log-level', LaunchConfiguration('velmwheel_log_level') ],
        # Running condition
        'condition' : IfCondition(PythonExpression([
            '"', LaunchConfiguration('with_slam'), '" == "true"',
            'and',
            '"', slam_source_scan_config, '" == "combined"',
        ])),

    }

# ============================================================= Launch ============================================================= #

def generate_slam_component_launch_description(
    executable,
    default_config_file = None
):

    '''Generates LaunchDescription object for the SLAM toolbox component node of the ecosystem of the WUT Velmwheel robot
    
    Parameters
    ----------
    executable : str
        name of the executable inside the slam_toolbox package
    default_config_file : str
        name of the default configuration file for the node; if @c None given the 
        confgiguation-file launch parameters is not added and the configuration file is
        not passed to the node; this may be absolute or relative to the @p package root
    '''

    # Selects source scans topic for SLAM based on the 'soruce_scan' parameter
    def select_slam_source_scan_topic():
        return PythonExpression([
            f'"/{ROBOT_NAME}/lidar_l/scan"',         ' if "', slam_source_scan_config, '" == "left"     else (',
            f'"/{ROBOT_NAME}/lidar_r/scan"',         ' if "', slam_source_scan_config, '" == "right"    else (',
            f'"/{ROBOT_NAME}/lidars/scan"',          ' if "', slam_source_scan_config, '" == "both"     else (',
            f'"/{ROBOT_NAME}/lidars/scan/combined"',                                                         ')))'
        ])

    # Generate descriptor of the SLAM node launch
    slam_descriptor = generate_component_launch_descriptor(
        
        # Executable to be run
        package = 'slam_toolbox',
        executable = executable,
        name = 'slam',

        # Default configuation file
        default_config_file = 
            str(get_package_share_path('slam_toolbox') / 'config' / default_config_file)
                if default_config_file is not None else None,

        # Common parameters for all configruation of the SLAM (overwrite config file)
        params = [

            { 'odom_frame':  'odom'                    },
            { 'map_frame':   'map'                     },
            { 'base_frame': f'{ROBOT_NAME}'            },
            { 'scan_topic': select_slam_source_scan_topic() },
            
        ],

        # Common remaps for all configruation of the SLAM
        remaps = [

            # Outputs
            ( '/map',                              f'/{ROBOT_NAME}/slam/map',                ),
            ( '/map_metadata',                     f'/{ROBOT_NAME}/slam/map_metadata'        ),
            ( '/slam_toolbox/graph_visualization', f'/{ROBOT_NAME}/slam/graph_visualization' ),
            ( '/slam_toolbox/scan_visualization',  f'/{ROBOT_NAME}/slam/scan_visualization'  ),
            
        ]
        
    )

    # Return launch description
    return LaunchDescription([
        
        # Launch arguments
        DeclareLaunchArgument(**declare_slam_source_scan_config_description),
        # Conversion node
        Node(**generate_scans_converter_descriptor()),
        # Description of the SLAM node
        *slam_descriptor

    ])
