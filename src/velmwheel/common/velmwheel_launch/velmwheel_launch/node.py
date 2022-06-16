# ====================================================================================================================================
# @file       node.py
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Thursday, 7th April 2022 5:18:37 am
# @modified   Wednesday, 15th June 2022 10:23:57 pm
# @project    engineering-thesis
# @brief      Set of helper functions utilized by launchfile across ecosystem of the WUT Velmwheel robot
#    
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Imports ============================================================ #

# ROS includes
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PythonExpression
from launch.actions import DeclareLaunchArgument, Shutdown, LogInfo
from launch.conditions import IfCondition
# Ament includes
from ament_index_python.packages import get_package_share_path
# Private includes
from velmwheel_model.params import ROBOT_NAME
from launch_common.arguments import declare_launch_argument

# =================================================== Launching system component =================================================== #

def generate_component_launch_descriptor(
    package,
    executable,
    name,
    default_config_file = None,
    default_on = 'false',
    params = [],
    remaps = [],
    additional_node_config = {},
    additional_node_args = [],
    prefix = None,
    additional_conditions = [],
    default_required = False
):

    """Generates array containing arguments for LaunchDescription object for the 
    typical component node of the ecosystem of the WUT Velmwheel robot with given remaps 
    and launch-parameters that allow configuring 'use_sim_time' parameter of the node as 
    well as set node's parameters config file

    Parameters
    ----------
    package : str
        name of the component's package
    executable : str
        name of the component's executable inside the @p package
    namee : str
        name of the component 
    default_config_file : str, optional
        name of the default configuration file for the node; if `None` given the 
        confgiguation-file launch parameters is not added and the configuration file is
        not passed to the node; this may be absolute or relative to the `package` root
    default_on : str, optional
        default value of the 'with_*' launch parameter
    params : list, optional
        list of additional node's parameters to be passed to `launch_ros.actions.Node`
        (are passed after the configuration file to overwrite it's settings)
    remaps : list, optional
        list of node's remaps to be passed to `launch_ros.actions.Node`
    additional_node_config : dict, optional
        dictionary of additional key-values settings to be passed to 
        `launch_ros.actions.Node` (appended at the endo of the Node's descriptor)
    additional_node_args : list, optional
        additional execution arguments that will be **appended** to the default list
    prefix : str
        a set of commands/arguments to preceed the cmd, used for things like gdb/valgrind
        and defaults to the LaunchConfiguration called 'launch-prefix'
    additional_conditions : list, optional
        additional conditions for the node to be run given as a list passed to PythonExpression
        substitution type (e.g. ["'", config_ref, " == 'true'"])
    default_required : bool
        each node will have it's dedicated *_required launch option that - if set to true -
        will trigger Shutdown() action 'on_exit'; this parameter sets default value for the
        argument
        
    """

    # ---------------------------------------- Arguments ----------------------------------------- 

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
        'name':          'with_' + name,
        'default_value': default_on,
        'description':   f'If set to "true", the {name} node will be run'
    })

    # Launch argument: flag stating whether launch should shutdown when the node exits
    declare_is_required_config_description, is_required_config = declare_launch_argument({
        'name':          name + '_required',
        'default_value': 'true' if default_required else 'false',
        'description':   f'If set to "true", the launch will run Shutdown() at node\'s exit'
    })

    # Launch argument: configruation of the node
    declare_config_config_description, config_config = declare_launch_argument({
        'name':          name + '_config',
        'default_value': '',
        'description':   f'Path to the YAML configuration file for the {name} node'
    })

    # ------------------------------------ Default arguments ------------------------------------- 

    # Set default config file (if given)
    if default_config_file is not None:
        if default_config_file.startswith("/"):
            declare_config_config_description['default_value'] = default_config_file
        else:
            declare_config_config_description['default_value'] = str(get_package_share_path(package) / default_config_file)

    # Prepare additional condition
    additional_conditions = additional_conditions if len(additional_conditions) != 0 else [ 'True' ]

    # ------------------------------------- Node description ------------------------------------- 

    # Creates description of the node's run
    def make_node_description(condition):

        # Description of the `base_controller` node
        node_description = {

            # Controller node
            'package': package,
            'executable': executable,
            'name': name,
            # Robot's namespace node
            'namespace': f'/{ROBOT_NAME}',
            # Output config
            'output': 'both',
            'emulate_tty': True,
            # Node's parameters
            'parameters': [

                # Simulation configuration
                { 'use_sim_time': use_sim_time_config },
                # Other parameters
                *params

            ],

            # Remap I/O topics
            'remappings': remaps,
            
            # Execution prefix
            'prefix': prefix,
            # Node's log level
            'arguments': [ '--ros-args', '--log-level', velmwheel_log_level_config, *additional_node_args ],
            # Running condition
            'condition' : IfCondition(PythonExpression([

                # Basic run condition
                "'", should_run_config, "' == 'true'", " and ",
                # Predefined run condition
                *condition, " and ",
                # Additonal run condition
                *additional_conditions
                
            ])),

            # Additional config
            **additional_node_config,
        }

        
        # Add parameters config file if requested
        if default_config_file is not None:
            node_description['parameters'].insert(0, config_config)

        return node_description

    # ------------------------------------ Requirement config ------------------------------------

    # Make description of the node's run when the node is required
    node_description_required = make_node_description(["'", is_required_config, "' == 'true'"])
    # Add on_exit action
    node_description_required['on_exit']=Shutdown()

    # Make description of the node's run when the node is NOT required
    node_description_non_required = make_node_description(["'", is_required_config, "' != 'true'"])

    # ------------------------------------ Launch description ------------------------------------

    # Description of the launch
    launch_description = [
        
        # Arguments
        DeclareLaunchArgument(**declare_velmwheel_log_level_description),
        DeclareLaunchArgument(**declare_use_sim_time_config_description),
        DeclareLaunchArgument(**declare_should_run_config_description),
        DeclareLaunchArgument(**declare_is_required_config_description),
        DeclareLaunchArgument(**declare_config_config_description),
        # Nodes
        Node(**node_description_required),
        Node(**node_description_non_required),
        
    ]

    # Append config-file launch parameter to the launch description if requested
    if default_config_file is not None:
        launch_description.append(DeclareLaunchArgument(**declare_config_config_description))

    # --------------------------------------------------------------------------------------------

    return launch_description

def generate_component_launch_description(
    package,
    executable,
    name,
    default_config_file = None,
    default_on = 'false',
    params = [],
    remaps = [],
    additional_node_config = {},
    additional_node_args = [],
    prefix = None,
    additional_conditions = [],
    default_required = False
) :

    """Generates LaunchDescription object for the typical component node of the
    ecosystem of the WUT Velmwheel robot with given remaps and launch-parameters
    that allow configuring 'use_sim_time' parameter of the node as well as set node's
    parameters config file

    Parameters
    ----------
    package : str
        name of the component's package
    executable : str
        name of the component's executable inside the @p package
    namee : str
        name of the component 
    default_config_file : str, optional
        name of the default configuration file for the node; if `None` given the 
        confgiguation-file launch parameters is not added and the configuration file is
        not passed to the node; this may be absolute or relative to the `package` root
    default_on : str, optional
        default value of the 'with_*' launch parameter
    params : list, optional
        list of additional node's parameters to be passed to `launch_ros.actions.Node`
        (are passed after the configuration file to overwrite it's settings)
    remaps : list, optional
        list of node's remaps to be passed to `launch_ros.actions.Node`
    additional_node_config : dict, optional
        dictionary of additional key-values settings to be passed to 
        `launch_ros.actions.Node` (appended at the endo of the Node's descriptor)
    additional_node_args : list, optional
        additional execution arguments that will be **appended** to the default list
    prefix : str
        a set of commands/arguments to preceed the cmd, used for things like gdb/valgrind
        and defaults to the LaunchConfiguration called 'launch-prefix'
    additional_conditions : list, optional
        additional conditions for the node to be run given as a list passed to PythonExpression
        substitution type (e.g. ["'", config_ref, " == 'true'"])
    default_required : bool
        each node will have it's dedicated *_required launch option that - if set to true -
        will trigger Shutdown() action 'on_exit'; this parameter sets default value for the
        argument
        
    """

    return LaunchDescription(generate_component_launch_descriptor(
        package                = package,
        executable             = executable,
        name                   = name,
        default_config_file    = default_config_file,
        default_on             = default_on,
        params                 = params,
        remaps                 = remaps,
        additional_node_config = additional_node_config,
        additional_node_args   = additional_node_args,
        prefix                 = prefix,
        additional_conditions  = additional_conditions,
        default_required       = default_required
    ))