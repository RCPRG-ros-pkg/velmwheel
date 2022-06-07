# ====================================================================================================================================
# @file       models.py
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Thursday, 19th May 2022 9:53:09 am
# @modified   Wednesday, 25th May 2022 12:21:41 am
# @project    engineering-thesis
# @brief      Set of helper functions utilized by launchfiles to include robots models
# 
# 
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================== Doc =============================================================== #

""" 

.. module:: models
   :platform: Unix
   :synopsis: Set of helper functions utilized by launchfiles to include robots models

.. moduleauthor:: Krzysztof Pierczyk <krzysztof.pierczyk@gmail.com>

"""

# ============================================================= Imports ============================================================ #

# System imports
from os import path
# Ament imports
from ament_index_python.packages import get_package_share_directory

# =========================================================== Definitions ========================================================== #

# ---------------------------------------------------------------------------------------
# @brief Helper function returning path to the URDF mdoel of the robot residing in the
#   /urdf subdirectory of the given @p package 
# ---------------------------------------------------------------------------------------
def get_urdf_source(package, model):

    """Helper function returning path to the URDF mdoel of the robot residing in the
    `/urdf` subdirectory of the given `package` 

    Parameters
    ----------
    package : str
        name of the target package
    model : str
        name of the target model file

    Returns
    -------
    model_path : str
        absolute path to the requested model file

    """
    
    return path.join( get_package_share_directory(package), f'urdf/{model}' )

# ================================================================================================================================== #
