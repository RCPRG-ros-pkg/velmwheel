# ====================================================================================================================================
# @file       doc.py
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Wednesday, 25th May 2022 12:21:36 pm
# @modified   Wednesday, 25th May 2022 4:05:48 pm
# @project    engineering-thesis
# @brief      General utilities related to configuring Sphinx documentation
# 
# 
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================== Doc =============================================================== #

""" 

.. module:: doc
   :platform: Unix
   :synopsis: General utilities related to configuring Sphinx documentation

.. moduleauthor:: Krzysztof Pierczyk <krzysztof.pierczyk@gmail.com>

"""

# ============================================================ Imports ============================================================= #

import os
from ament_index_python.packages import get_package_share_directory

# ============================================================ Script ============================================================== #

def find_package_doc(package_name, location=None, type="html"):

    """Finds absolute path to the index file of the package's documentation
    
    Parameters
    ----------
    package_name : str
        name of the target package
    location : str or None, optionl
        location of the documentation relative to the installation directory; 
        if `None` given, the default location is set to 
        `share/<package_name>/doc/sphinx/<type>`
    type : type, optional
        type of the target documentation

    Returns
    -------
    path : str
        path to the found documentation dircetory

    Raises
    ------
    PackageNotFoundError
        if package is not found
    FileNotFoundError
        if documentation is not found
    
    """
    
    # Calculate default location
    if location is None:
        location = f'share/{package_name}/doc/sphinx/{type}'

    # Find path to the package
    package_path = get_package_share_directory(package_name)
    # Calculate path to the documentation
    link_path = f'{package_path}/../../{location}'

    # Check if documentation exists
    if not os.path.exists(link_path):
        raise FileNotFoundError(f'Could not find the documentation for {package_name} package ({link_path})')

    return link_path

def make_package_extlinks_doc_link(package_name, link_name=None, user_text=None, location=None, type="html"):

    """ Creates dictionary describing Sphinx link to the external
    documentation of the ament package named `package_name`

    Parameters
    ----------
    package_name : str
        name of the target package
    link_name : str, optional
        name of the link as referenced from RST docs; if `None`, `link_name` 
        will be set to `package_name`
    user_text : str, optional
        user text describing the link
    location : str or None, optionl
        location of the documentation relative to the installation directory; 
        if `None` given, the default location is set to 
        `share/<package_name>/doc/sphinx/<type>/index.<type>`
    type : type, optional
        type of the target documentation

    Returns
    -------
    link_description : dict
        dictionary describing Sphinx-compatible link to the project

    Raises
    ------
    PackageNotFoundError
        if package is not found
    FileNotFoundError
        if documentation is not found

    See Also
    --------
    * `Sphinx: ext.extlinks <https://www.sphinx-doc.org/en/master/usage/extensions/extlinks.html>`_
    * `Auxiliary example <https://stackoverflow.com/questions/61239434/can-the-extlinks-extension-to-sphinx-be-configured-to-show-the-url>`_
    
    """

    # Set default link_name
    link_name = link_name if link_name is not None else package_name

    # Return link
    return { link_name : (find_package_doc(package_name, location, type) + f'/index.{type}', user_text) }


def make_package_intersphinx_doc_link(package_name, link_name=None, location=None, type="html"):

    """ Creates dictionary describing intersphinx link to the external
    documentation of the ament package named `package_name`

    Parameters
    ----------
    package_name : str
        name of the target package
    link_name : str, optional
        name of the link as referenced from RST docs; if `None`, `link_name` 
        will be set to `package_name`
    location : str or None, optionl
        location of the documentation relative to the installation directory; 
        if `None` given, the default location is set to 
        `share/<package_name>/doc/sphinx/<type>`
    type : type, optional
        type of the target documentation

    Returns
    -------
    link_description : dict
        dictionary describing Sphinx-compatible link to the project

    Raises
    ------
    PackageNotFoundError
        if package is not found
    FileNotFoundError
        if documentation is not found

    See Also
    --------
    `Sphinx: ext.intersphinx <https://www.sphinx-doc.org/en/master/usage/extensions/intersphinx.html>`_
    
    """

    # Set default link_name
    link_name = link_name if link_name is not None else package_name

    # Return descriptor
    return { link_name : (find_package_doc(package_name, location, type), None) }

# ================================================================================================================================== #
