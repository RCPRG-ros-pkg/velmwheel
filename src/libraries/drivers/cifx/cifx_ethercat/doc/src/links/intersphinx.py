# ====================================================================================================================================
# @file       intersphinx.py
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Wednesday, 25th May 2022 11:48:20 am
# @modified   Wednesday, 25th May 2022 10:33:16 pm
# @project    engineering-thesis
# @brief      Definitions of intersphinx links refering external documentations of libraries used by the `cifx-ethercat` package
# 
# 
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================== Doc =============================================================== #

""" 

.. module:: links.intersphinx
   :platform: Unix
   :synopsis: Definitions of intersphinx links refering external documentations of libraries used by the `cifx-ethercat` package

.. moduleauthor:: Krzysztof Pierczyk <krzysztof.pierczyk@gmail.com>

"""

# ============================================================= Imports ============================================================ #

from doc_common.sphinx import make_package_intersphinx_doc_link

# =========================================================== Definitions ========================================================== #

# List of links to documentations of subpackages
intersphinx_mapping = {
    **make_package_intersphinx_doc_link( 'ethercat'     ),
    **make_package_intersphinx_doc_link( 'cifx_toolkit' )
}

# ================================================================================================================================== #
