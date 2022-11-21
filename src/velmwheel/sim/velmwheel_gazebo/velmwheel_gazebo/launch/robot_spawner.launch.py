# ====================================================================================================================================
# @file       gazebo.launch.py
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Friday, 4th March 2022 5:57:12 pm
# @modified   Tuesday, 5th July 2022 5:17:28 am
# @project    engineering-thesis
# @brief      Launchfile running Gazebo-specific roslaunch file for spawning robot's model in the simulated world
# 
# 
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Imports ============================================================ #

# ROS launch imports
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_context import LaunchContext
# Private imports
from velmwheel_model.params import ROBOT_NAME
from launch_common.arguments import declare_launch_argument

# =========================================================== Arguments ============================================================ #

# Launch argument: version of the robot's base plugin to be loaded
declare_base_plugin_config_description, base_plugin_config = declare_launch_argument({
    'name':          'base_plugin',
    'default_value': 'real',
    'description':   'Sets version of the robot\'s base plugin to be loaded',
    'choices':       [ 'real', 'ideal' ]
})

# Launch argument: output mode of the LIDAR drivers
declare_laser_output_mode_config_description, laser_output_mode_config = declare_launch_argument({
    'name':          'laser_output_mode',
    'default_value': 'separate',
    'description':   'Publishing mode of the node indicating what topics it publishes to (@see lidar_gazebo.hpp)',
    'choices':       [ 'separate', 'common', 'both' ]
})

# Launch argument: Configuration of the LIDARs rays visualization
declare_laser_visualize_config_description, laser_visualize_config = declare_launch_argument({
    'name':          'laser_visualize',
    'default_value': 'false',
    'description':   'Configuration of the LIDARs rays visualization',
    'choices':       [ 'true', 'false' ]
})

# ========================================================== Configuration ========================================================= #

# Topic broadcasting SDF-tuned version of the robot description
SDF_ROBOT_DEXCRIPTION_TOPIC = "robot_description_tuned"

# ======================================================= Nodes descriptions ======================================================= #

def fine_tunner_node_description(plugin_impl, lidar_output_mode, laser_visualize):

    """Helper function producing a description of the node running 'regex_forwarder'
    utility that modifies robot's description published on the /robot_description
    topic and republishes it on the custom topic tunning its content for Gazebo's
    URDF -> SDF translator

    Parameters
    ----------
    plugin_impl : str
        implementation of the plugin conditioning running of the node; either 'real' or
        'ideal'
    lidar_output_mode : str
        either 'separate', 'common' or 'both'; determines topic that the LIDAR plugins
        are publishing data into
    laser_visualize : str
        either 'true' or 'false' string indicating whether LIDARs ray visualization should 
        be enabled

    Note
    ----
    The function can produce at most 6 different Node's descriptions from which
    only one has it's activation condition met. Such a solution is in fact ugly but
    is the only way to go at the moment due to limited functionality of substitution
    mechanism which makes it unable to use @ref PythonExpression substitution inside the
    definition of the 'replacements' argument. Although it's type is aimed to be
    STRING_ARRAY, the substitution mechanism treats mixed list of strings and
    @ref Substitution objects as a target to be resolved into a single string. In result
    one cannot use a single Node definition with the 'replacements' argument defined 
    with Substitution and instead needs to produce one one for each combination of
    of launchfile's arguments.
    
    Todo
    ----
    Fixme

    """

    return {

        'package': 'regex_forwarder',
        'executable': 'regex_forwarder',
        'name': 'robot_description_tuner',
        'output': 'both',
        'namespace': f'/{ROBOT_NAME}',

        # Parameters
        'parameters': [

            # Parss replacements in the xacro-generated description to prepare better material for Gazebo's URDF-SDF translator
            {
                'replacements' : [

                    # ---------------------------------------------------------------
                    # @note The obvious solution to 'package://' inconsistency
                    #    would be provide paths to meshes via xacro's 
                    #    $(find_package...) construct. Unfortunatelly, 'package://'
                    #    expression is not only resolved into absolute path to the
                    #    'velmwheel_model' package, but also serves to 'rviz2' to
                    #    add appropriate directories to the 'ResourceGroupManager'
                    #    for Ogre software to properly render 3D meshes. For this 
                    #    reason the 'package://' construct must be preserved in the
                    #    'velmwheel_model' package and 'velmwheel_gazebo' package
                    #    needs to switch these paths on the runtime to give valid
                    #    paths for Gazebo's URDF->SDF converter 
                    # ---------------------------------------------------------------

                    # Prepare proper paths for media (meshes)
                    'package://velmwheel_model/:=file://',
                    # Switch from STL to DAE to get better colors in the Gazebo
                    '.stl"/>:=.dae"/>',
                    # Select implementation of the base's plugin
                    f'__base_plugin_impl__:={plugin_impl}',
                    # Select LIDARs output's remap (either 'lidar_x/scan' or 'lidars/scan)
                    f'__lidar_out_mode__:={lidar_output_mode}',
                    # Select LIDARs visualization mode
                    f'__lidar_visualize__:={laser_visualize}'
                    
                ]
            },
            
            # Override QoS of in/out topics to fit /robot_description interface
            { f'qos_overrides./{ROBOT_NAME}/robot_description.subscription.durability':          'transient_local' },
            { f'qos_overrides./{ROBOT_NAME}/{SDF_ROBOT_DEXCRIPTION_TOPIC}.publisher.durability': 'transient_local' }
        ],
        # Remap input and output channels of the raplcer
        'remappings': [
            ('~/in',  f'/{ROBOT_NAME}/robot_description'),
            ('~/out', f'/{ROBOT_NAME}/{SDF_ROBOT_DEXCRIPTION_TOPIC}'),
        ],

        # Run condition
        'condition': IfCondition(PythonExpression([ 
            '"', base_plugin_config,       f'" == "{plugin_impl}"',
            ' and ',
            '"', laser_output_mode_config, f'" == "{lidar_output_mode}"',
            ' and ',
            '"', laser_visualize_config,   f'" == "{laser_visualize}"'
        ]))

    }
    
# Description of the node spawning a Velmwheel model in the simulation
def spawn_velmwheel_node_description():
    return {

        # Node's description
        'package': 'gazebo_ros',
        'executable': 'spawn_entity.py',
        # Model's parameters
        'arguments': [
            '-timeout',         '45.0',
            '-entity',          'velmwheel',
            '-robot_namespace', f'/{ROBOT_NAME}',
            '-topic',           f'/{ROBOT_NAME}/{SDF_ROBOT_DEXCRIPTION_TOPIC}',
            '-x',                '3.0',
            '-y',                '2.0'
        ],
        # Output configuration
        'output': 'screen',

    }

# ======================================================== Nodes descriptors ======================================================= #

# Define fine-tuner runners
fine_tunner_node = [ fine_tunner_node_description( plugin_impl, lidar_output_mode, laser_visualize )
    for plugin_impl       in declare_base_plugin_config_description['choices']
    for lidar_output_mode in declare_laser_output_mode_config_description['choices']
    for laser_visualize   in declare_laser_visualize_config_description['choices']
]

# Description of the node spawning a Velmwheel model in the simulation
spawn_velmwheel_node = [ spawn_velmwheel_node_description() ]

# =========================================================== Description ========================================================== #

# Description of the launch
launch_description = [
    
    # Arguments
    DeclareLaunchArgument(**declare_base_plugin_config_description),
    DeclareLaunchArgument(**declare_laser_output_mode_config_description),
    DeclareLaunchArgument(**declare_laser_visualize_config_description),

    # Nodes
    *[ Node(**description) for description in fine_tunner_node     ],
    *[ Node(**description) for description in spawn_velmwheel_node ]
    
]

# ============================================================= Launch ============================================================= #

def generate_launch_description():
    return LaunchDescription(launch_description)
