# ====================================================================================================================================
# @file       arguments.py
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Thursday, 19th May 2022 9:53:09 am
# @modified   Wednesday, 25th May 2022 12:07:09 am
# @project    engineering-thesis
# @brief      Set of helper functions utilized by launchfiles to declare and use launch arguments 
# 
# 
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================== Doc =============================================================== #

""" 

.. module:: arguments
   :platform: Unix, Windows
   :synopsis: Set of helper functions utilized by launchfiles to declare and use launch arguments 

.. moduleauthor:: Krzysztof Pierczyk <krzysztof.pierczyk@gmail.com>

"""

# ============================================================= Imports ============================================================ #

from launch.substitutions import LaunchConfiguration

# ============================================================= Helpers ============================================================ #

def declare_launch_argument(description : dict):

    """ Auxiliary wrapper around :obj:`launch.substitutions.LaunchConfiguration` providing 
    a clean way to avoid boilerplate code when declaring both description of the launch
    parameter and substitution-type reference to it's value

    Parameters
    ----------
    description : dict
        dictionary containing arguments for :obj:`launch.actions.DeclareLaunchArgument`

    Returns
    -------
    description : dict 
        source `description` passed to the function
    substitution : launch.substitutions.LaunchConfiguration
        substitution object referenceing runtime value of the launch argument described by
        `description`

    Examples
    --------
    >>> declare_use_sim_time_config_description, use_sim_time_config = declare_launch_argument({
    ...     'name':          'use_sim_time',
    ...     'default_value': 'false',
    ...     'description':   'Set to "true" to use simulated time'
    ... })
    >>> declare_use_sim_time_config_description
    {'name': 'use_sim_time', 'default_value': 'false', 'description': 'Set to "true" to use simulated time'}
    >>> use_sim_time_config
    <launch.substitutions.launch_configuration.LaunchConfiguration object at 0x7fe09ed04190>
    
    """

    return description, LaunchConfiguration(description['name'])

# ================================================================================================================================== #
