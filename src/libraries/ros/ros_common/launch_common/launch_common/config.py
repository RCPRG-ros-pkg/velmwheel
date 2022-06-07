# ====================================================================================================================================
# @file       config.py
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Thursday, 19th May 2022 9:53:09 am
# @modified   Wednesday, 25th May 2022 12:17:02 am
# @project    engineering-thesis
# @brief      Set of helper functions utilized by launchfiles to manipulate config files
# 
# 
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================== Doc =============================================================== #

""" 

.. module:: config
   :platform: Unix
   :synopsis: Set of helper functions utilized by launchfiles to manipulate config files

.. moduleauthor:: Krzysztof Pierczyk <krzysztof.pierczyk@gmail.com>

"""
# ============================================================= Imports ============================================================ #

# System imports
from os import path
# Ament imports
from ament_index_python.packages import get_package_share_directory

# =========================================================== Definitions ========================================================== #

# ---------------------------------------------------------------------------------------
# @brief  
# ---------------------------------------------------------------------------------------
def get_config_source(package, config):

    """Helper function returning path to the `config` file residing in the `/config`
    subdirectory of the given `package`

    Parameters
    ----------
    package : str
        name of the target package
    config : str
        name of the target config file

    Returns
    -------
    config_path : str
        absolute path to the requested config file

    """
    
    return path.join( get_package_share_directory(package), f'config/{config}' )

# ================================================================================================================================== #
