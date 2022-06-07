# ====================================================================================================================================
# @file       laser_odom.launch.py
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Friday, 4th March 2022 5:57:12 pm
# @modified   Thursday, 26th May 2022 12:50:46 am
# @project    engineering-thesis
# @brief      Launchfile running the nodes responsible for LIDAR-based odometry
# @details 
#
#                                      ______________________                               ______________________ 
#                                     |                      |                             |                      |
#               ---- lidars/scan ---> | laser_scan_converter | ------ lidars/clouds --.--> | laser_scan_assembler |
#                                     |______________________|                        |    |______________________|
#                                                                                   [3/4]              |           
#               ---- lidar_l/scan ---------------[2]-----------------------------.    |                |           
#                                                                                |    |                |           
#                                                                                |    |               [5]          
#               ---- lidar_r/scan ---------------[1]------------------------.    |    |                |           
#                                                                           |    |    |                |           
#                                                                           |    |    |                |           
#               ------ imu/out ----------------------------------------.    |    |    |           lidars/cloud     
#                                                                      |    |    |    |                |           
#                                                                      |    |    |    |                |           
#               ------- odom -------------------------------------.    |    |    |    |                |           
#                                                                 |    |    |    |    |                |           
#                                                                 |    |    |    |    |                |           
#                                                               __v____v____v____v____v__              |           
#                                                              |                         |             |           
#                                                              |                         |             |           
#                                                              |                         |             |           
#                     [EKF, bias estimator] <-- laser_odom --- |    laser_scan_matcher   |<------------'           
#                                                              |                         |                         
#                                                              |                         |                         
#                                                              |_________________________|                         
#
# @note [x] refer to optional connections depending on the configuration. LIDAR data can be provideed for the laser_scan_matcher
#    from one of five sources
#
#       * laser scan from the right LIDAR [1]
#       * laser scan from the left LIDAR [2]
#       * point cloud from the right LIDAR [3]
#       * point cloud from the left LIDAR [4]
#       * point cloud assembled from data from both LIDARs [5]
#
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Imports ============================================================ #

# ROS imports
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import PythonExpression
# Ament imports
from ament_index_python import get_package_share_path
# Private imports
from velmwheel_model.params import ROBOT_NAME
from launch_common.arguments import declare_launch_argument
from launch_common.config import get_config_source

# =========================================================== Arguments ============================================================ #

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

# Launch argument: whether to run laser-scan-to-cloud conversion
declare_with_laser_conversion_config_description, with_laser_conversion_config = declare_launch_argument({
    'name':          'with_laser_conversion',
    'default_value': 'true',
    'description':   'Set to "false" to not run laser-scan-to-cloud conversion nodes'
})

# Launch argument: whether to run laser-based odometry
declare_with_laser_odom_config_description, with_laser_odom_config = declare_launch_argument({
    'name':          'with_laser_odom',
    'default_value': 'true',
    'description':   'Set to "false" to not run laser-odometry node'
})

# Launch argument: laser scan matcher source mode
declare_mode_description, mode_config = declare_launch_argument({
    'name':          'mode',
    'default_value': 'combined',
    'description':
        "Mode indicating LIDAR measurements used by the laser_scan_matcher. 'right', 'left', "       +
        "and 'combined' refer to providing laser_scan_matcher with cloud of points generated from "  +
        "left, right or both LIDAR sensors respectively. 'left_scan' and 'right_scan' modes result " +
        "in sourcing laser-base odomtery node with raw LaserScan data generated from left or right " +
        "LIDAR sensor",
    'choices': [ 'left', 'right', 'combined', 'left_scan', 'right_scan' ]
})

# Launch argument: laser scan matcher source mode
declare_combining_mode_description, combining_mode_config = declare_launch_argument({
    'name':          'combining_mode',
    'default_value': 'batch',
    'description':
        "Assembly mode for the node combining points clouds incoming from LIDAR scaners. 'batch' " +
        "mode means, that the combined cloud will be published after at least one fresh update of " +
        "measurement from each LIDAR has been received since the last publication. If 'continuous' " +
        "mode is set, the combined cloud will be published at each of measurements arrival",
    'choices': [ 'batch', 'continuous' ]
})

# Launch argument: configruation of the odom node
declare_laser_odom_config_description, laser_odom_config_config = declare_launch_argument({
    'name':          'laser_odom_config',
    'default_value': get_config_source('laser_scan_matcher', 'laser_scan_matcher.yaml'),
    'description':   'Path to the YAML configuration file for the odom node'
})

# Launch argument: reference frame of the odom-base pose
declare_laser_odom_reference_description, laser_odom_reference_config = declare_launch_argument({
    'name':          'laser_odom_reference',
    'default_value': 'odom',
    'description':   'TF reference frame of the odom-base pose'
})

# =========================================================== Predicates =========================================================== #

# Substitution expression resolving to "true" when the source data matching mode is set to @p mode
def scan_source_condition(mode):
    return IfCondition( PythonExpression([
        '"', with_laser_conversion_config, '" == "true"'
        ' and ',
        '"', mode_config, f'" == "{mode}"'
    ]) )

# Substitution expression resolving to "true" when the source data matching mode is set to @p mode
def scan_matcher_condition(mode):
    return IfCondition( PythonExpression([
        '"', with_laser_odom_config, '" == "true"'
        ' and ',
        '"', mode_config, f'" == "{mode}"'
    ]) )

# ======================================================= Nodes descriptions ======================================================= #

def laser_scan_converter_description(input_topic, condition):

    """Produces description of the node converting laser scan to the point cloud (`laser_scan_converter` node)"""

    return {

        # Node
        'package': 'laser_scan_converter',
        'executable': 'laser_scan_converter_node',
        'name': 'laser_scan_converter',
        'namespace': f'/{ROBOT_NAME}',
        # Output config
        'output': 'both',
        'emulate_tty': True,

        # Node's parameters
        'parameters': [

            # Simulation configuration
            { 'use_sim_time': use_sim_time_config }
        ],

        # Remap I/O topics
        'remappings': [

            # Inputs
            ( 'scan', input_topic ),
            # Outputs
            ( 'cloud', f'/{ROBOT_NAME}/lidars/cloud' ),

        ],

        # Node's log level
        'arguments': [ '--ros-args', '--log-level', velmwheel_log_level_config ],
        
        # Run only if Cloud-based scan matching is used
        'condition': condition
            

    }


def laser_scan_assembler_description():

    """Produces description of the node assembling point clouds (`laser_scan_assembler` node)"""
    
    return {

        # Node
        'package': 'laser_scan_assembler',
        'executable': 'laser_scan_assembler_node',
        'name': 'laser_scan_assembler',
        'namespace': f'/{ROBOT_NAME}',
        # Output config
        'output': 'both',
        'emulate_tty': True,
        
        # Parameters
        'parameters': [

            # Simulation configuration
            { 'use_sim_time': use_sim_time_config },
            # Reference frame of the resulting cloud
            { 'reference_frame': f'{ROBOT_NAME}' },
            # Frames to be combined
            { 'frames': [ 'lidar_l_scan', 'lidar_r_scan' ] },
            # Frames combining mode
            { 'combining_mode': combining_mode_config },
            # Points type
            { 'point_type': 'XYZI' }

        ],

        # Remap I/O topics
        'remappings': [

            # Inputs
            ( 'cloud_in', f'/{ROBOT_NAME}/lidars/cloud' ),
            # Outputs
            ( 'cloud_out', f'/{ROBOT_NAME}/lidars/cloud/combined' ),

        ],    

        # Node's log level
        'arguments': [ '--ros-args', '--log-level', velmwheel_log_level_config ],
        
        # Run condition
        'condition': scan_source_condition('combined')
        
    }


def laser_scan_matcher_description(use_cloud_input, input_remapping, condition):
    
    """Produces description of the `laser_scan_matcher` node"""

    return {

        # Node
        'package': 'laser_scan_matcher',
        'executable': 'laser_scan_matcher_node',
        'name': 'laser_scan_matcher',
        'namespace': f'/{ROBOT_NAME}',
        # Output config
        'output': 'both',
        'emulate_tty': True,
        
        # Parameters
        'parameters': [

            # Basic configruation
            laser_odom_config_config,
            # Simulation configuration
            { 'use_sim_time': use_sim_time_config },
            # Overwrites
            { 'reference_frame':      laser_odom_reference_config  },
            { 'base_frame':           f'{ROBOT_NAME}'              },
            { 'target_frame':         'laser_odom_pose'            },
            { 'use_cloud_input':      use_cloud_input              },
            { 'stamped_vel':          True                         },
            { 'publish_pose':         False                        },
            { 'publish_pose_stamped': True                         },
            { 'cloud_sorted':         False                        }
            
        ],

        # Remap I/O topics
        'remappings': [

            # Laser scan/cloud input
            input_remapping,
            # Auxiliary inputs
            ( 'pose_guess',              f'/{ROBOT_NAME}/laser_scan_matcher/pose_guess' ),
            ( 'imu',                     f'/{ROBOT_NAME}/imu/out'                       ),
            ( 'odom',                    f'/{ROBOT_NAME}/odom/encoders'                 ),
            ( 'velocity_stamped',        f'/{ROBOT_NAME}/base/velocity'                 ),
            # Outputs
            ( 'odom/laser/pose_stamped', f'/{ROBOT_NAME}/odom/laser/pose'               ),

        ],

        # Node's log level
        'arguments': [ '--ros-args', '--log-level', velmwheel_log_level_config ],

        # Run condition
        'condition': condition
        
    }

# ====================================================== Nodes instantiations ====================================================== #

# Description of the node converting laser scan to the point cloud (`laser_scan_converter` node)
laser_scan_converter_descriptions = [
    laser_scan_converter_description(f'/{ROBOT_NAME}/lidars/scan',  scan_source_condition('combined') ),
    laser_scan_converter_description(f'/{ROBOT_NAME}/lidar_r/scan', scan_source_condition('right')    ),
    laser_scan_converter_description(f'/{ROBOT_NAME}/lidar_l/scan', scan_source_condition('left')     )
]

# Description of the node assembling point clouds (`laser_scan_assembler` node)
laser_scan_assembler_descriptions = [
    laser_scan_assembler_description()
]

# Description of the `laser_scan_matcher` node
laser_scan_matcher_descriptions = [
    laser_scan_matcher_description( True,  ( 'cloud', f'/{ROBOT_NAME}/lidars/cloud'          ), scan_matcher_condition('left')       ),
    laser_scan_matcher_description( True,  ( 'cloud', f'/{ROBOT_NAME}/lidars/cloud'          ), scan_matcher_condition('right')      ),
    laser_scan_matcher_description( True,  ( 'cloud', f'/{ROBOT_NAME}/lidars/cloud/combined' ), scan_matcher_condition('combined')   ),
    laser_scan_matcher_description( False, ( 'scan',  f'/{ROBOT_NAME}/lidar_l/scan'          ), scan_matcher_condition('left_scan')  ),
    laser_scan_matcher_description( False, ( 'scan',  f'/{ROBOT_NAME}/lidar_r/scan'          ), scan_matcher_condition('right_scan') )
]

# =========================================================== Description ========================================================== #

# Description of the launch
launch_description = [
    
    # Arguments
    DeclareLaunchArgument(**declare_velmwheel_log_level_description),
    DeclareLaunchArgument(**declare_use_sim_time_config_description),
    DeclareLaunchArgument(**declare_with_laser_odom_config_description),
    DeclareLaunchArgument(**declare_with_laser_conversion_config_description),
    DeclareLaunchArgument(**declare_mode_description),
    DeclareLaunchArgument(**declare_combining_mode_description),
    DeclareLaunchArgument(**declare_laser_odom_config_description),
    DeclareLaunchArgument(**declare_laser_odom_reference_description),
    # Nodes
    *[ Node(**description) for description in laser_scan_converter_descriptions ],
    *[ Node(**description) for description in laser_scan_assembler_descriptions ],
    *[ Node(**description) for description in laser_scan_matcher_descriptions   ],
    
]

# ============================================================= Launch ============================================================= #

def generate_launch_description():
    return LaunchDescription(launch_description)
