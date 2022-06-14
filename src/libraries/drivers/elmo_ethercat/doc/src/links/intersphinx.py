# ====================================================================================================================================
# @file       intersphinx.py
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Wednesday, 25th May 2022 11:48:20 am
# @modified   Tuesday, 31st May 2022 1:37:13 pm
# @project    engineering-thesis
# @brief      Definitions of intersphinx links refering documentations of component packages of the `ros-common` meta-package
# 
# 
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================== Doc =============================================================== #

""" 

.. module:: links.intersphinx
   :platform: Unix
   :synopsis: Definitions of intersphinx links refering documentations of component packages of the `imu-ethercat` meta-package

.. moduleauthor:: Krzysztof Pierczyk <krzysztof.pierczyk@gmail.com>

"""

# ============================================================= Imports ============================================================ #

from doc_common.sphinx import make_package_intersphinx_doc_link

# =========================================================== Definitions ========================================================== #

# List of links to documentations of subpackages
intersphinx_mapping = {
    **make_package_intersphinx_doc_link( 'ethercat' ),
}

# ================================================================================================================================== #
