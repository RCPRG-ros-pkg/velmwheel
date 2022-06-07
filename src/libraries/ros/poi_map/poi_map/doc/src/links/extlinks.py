# ====================================================================================================================================
# @file       extlinks.py
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Wednesday, 25th May 2022 11:48:20 am
# @modified   Wednesday, 25th May 2022 6:04:28 pm
# @project    engineering-thesis
# @brief      Definitions of extlinks links refering documentations of component packages of the `poi-map` meta-package
# 
# 
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================== Doc =============================================================== #

""" 

.. module:: links.external
   :platform: Unix
   :synopsis: Definitions of extlinks links refering documentations of component packages of the `poi-map` meta-package

.. moduleauthor:: Krzysztof Pierczyk <krzysztof.pierczyk@gmail.com>

"""

# ============================================================= Imports ============================================================ #

from doc_common.sphinx import make_package_extlinks_doc_link

# =========================================================== Definitions ========================================================== #

# List of links to documentations of subpackages
extlinks = {
    **make_package_extlinks_doc_link( 'poi_map_lib'  ),
    **make_package_extlinks_doc_link( 'poi_map_msgs' )
}

# ================================================================================================================================== #
