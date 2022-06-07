# ====================================================================================================================================
# @file       includes.py
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Thursday, 19th May 2022 9:53:09 am
# @modified   Wednesday, 25th May 2022 12:20:52 am
# @project    engineering-thesis
# @brief      Set of helper functions utilized by launchfiles to including sub-launchfiles
# 
# 
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================== Doc =============================================================== #

""" 

.. module:: includes
   :platform: Unix
   :synopsis: Set of helper functions utilized by launchfiles to including sub-launchfiles

.. moduleauthor:: Krzysztof Pierczyk <krzysztof.pierczyk@gmail.com>

"""

# ============================================================= Imports ============================================================ #

# System imports
from os import path
# Launch imports
from launch.launch_description_sources import PythonLaunchDescriptionSource
# Ament imports
from ament_index_python.packages import get_package_share_directory

# =========================================================== Definitions ========================================================== #

# ---------------------------------------------------------------------------------------
# @brief Helper function returning @ref PythonLaunchDescriptionSource for the given
#    @p name launchfile residint in the @folder launch/components directory of the 
#    @p package
# ---------------------------------------------------------------------------------------
def get_launch_source(package, name):

    """Helper function returning :obj:`launch.launch_description_sources.PythonLaunchDescriptionSource`
    `name` launchfile residint in the `/launch` subdirectory of the `package`

    Parameters
    ----------
    package : str
        name of the target package
    name : str
        name of the target launch file

    Returns
    -------
    description : launch.launch_description_sources.PythonLaunchDescriptionSource
        launch description source of the requested launchfile

    """

    return PythonLaunchDescriptionSource([ 
        path.join( get_package_share_directory(package), f'launch/{name}' ) 
    ])

# ================================================================================================================================== #
