# ====================================================================================================================================
# @file       setup.py
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Wednesday, 2nd March 2022 12:18:17 pm
# @modified   Wednesday, 25th May 2022 11:16:52 pm
# @project    engineering-thesis
# @brief      Setup file of the package
# 
# 
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Imports ============================================================ #

from setuptools import setup

# ========================================================== Package setup ========================================================= #

# Package's name
package_name = 'regex_forwarder'

# Package's setup
setup(

    # Package's name
    name=package_name,
    # Package's version
    version='0.0.0',
    # Package's content
    packages=[ package_name ],
    # Package's files
    data_files=[
        ( 'share/ament_index/resource_index/packages', ['resource/' + package_name] ),
        ( 'share/' + package_name,                     ['package.xml']              ),
    ],

    # Package's requirements
    install_requires=['setuptools'],
    # Package's ZIP config
    zip_safe=True,

    # Package's maintainer
    maintainer='Krzysztof Pierczyk',
    # Package's maintainer's email
    maintainer_email='krzysztof.pierczyk@gmail.com',

    # Package's description
    description='Utility node enabling echoing string content of the ROS topic to another topic with regex replacements applied',
    # Package's license
    license='Apache License 2.0',

    # Package's entry points
    entry_points={
        'console_scripts': [
            'regex_forwarder = regex_forwarder.regex_forwarder:main',
        ],
    },
)
