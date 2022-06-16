# ====================================================================================================================================
# @file       ethercat_driver.py
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Thursday, 28th April 2022 12:33:32 pm
# @modified   Wednesday, 15th June 2022 10:21:49 pm
# @project    engineering-thesis
# @brief
#    
#    Launchfile for the EtherCAT driver submodule for the the WUT Velmwheel robot's driveline
#    
# @note This file is not designed to be run as a standalone launchfile but rather defines launch-descriptor generator that
#    can be used by a downstream package to setup driver's configuration for the given use-case. Reasoning behind this approach 
#    is that `velmwheel_ethercat_driver` operates basing on set of loadable plugins that provide actual processing interfaces
#    for slave devices present on the EtherCAT bus. As so, the raw driver has no informations about what ROS parameters or what
#    ROS topics (and so topics remappings) are required by the loaded plugins. These must be provided by the downstream launch
#    that knows what plugins will be loaded to the driver.
#
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Imports ============================================================ #

from velmwheel_launch.node import generate_component_launch_description

# ============================================================ Constants =========================================================== #

# Name of the node in the WUT Velmwheel robot system
node_name = 'ethercat_driver'

# ============================================================= Launch ============================================================= #

# ---------------------------------------------------------------------------------------
# @brief Provides launch description for the `velmwheel_ethercat_driver` node
#
# @param ...
#    keyword arguments passed to [1]
#
# @see [1] velmwheel_launch.node.generate_component_launch_description(...)
# ---------------------------------------------------------------------------------------
def generate_launch_description(
    params = [],
    remaps = [],
    additional_node_config = {},
    additional_conditions = [],
    default_required = False,
    prefix = None
):
    return generate_component_launch_description(
        
        # Executable to be run
        package = 'velmwheel_ethercat_driver',
        executable = 'velmwheel_ethercat_driver',
        name = node_name,
        
        # Default configuation file
        default_config_file = "config/ethercat_driver.yaml",

        # Make driver default-launched
        default_on = 'true',

        # Additional arguments
        default_required       = default_required,
        prefix                 = prefix,
        params                 = params,
        remaps                 = remaps,
        additional_node_config = additional_node_config,
        additional_conditions  = additional_conditions

    )

# ================================================================================================================================== #
