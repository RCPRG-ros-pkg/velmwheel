# ====================================================================================================================================
# @file       __init__.py
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Wednesday, 25th May 2022 11:45:45 am
# @modified   Wednesday, 25th May 2022 11:48:10 am
# @project    engineering-thesis
# @brief      Module providing definitions of external lins for package's documentation
# 
# 
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================== Doc =============================================================== #

""" Module providing definitions of external lins for package's documentation

.. moduleauthor:: Krzysztof Pierczyk <krzysztof.pierczyk@gmail.com>

"""

# ============================================================ Imports ============================================================= #

from os.path import dirname, basename, isfile
import glob

# ============================================================ Script ============================================================== #

# List all submodules in the module
modules = glob.glob(dirname(__file__)+"/*.py")
# Add them to 'import *' list of the module
__all__ = [ basename(f)[:-3] for f in modules if isfile(f)]

# ================================================================================================================================== #
