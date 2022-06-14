# ====================================================================================================================================
# @file       description.py
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Thursday, 28th April 2022 11:14:04 pm
# @modified   Monday, 30th May 2022 11:07:56 pm
# @project    engineering-thesis
# @brief      
#    
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Imports ============================================================ #

# Ament imports
from ament_index_python.packages import get_package_share_path
# Private imports
from velmwheel_model.params import ROBOT_NAME

# ============================================================ Constants =========================================================== #

# Name of the plugin
plugin_name = 'velmwheel_base_driver::ImuDriver'

# Path to default parameters file
default_parameters = str(get_package_share_path('velmwheel_imu_driver') / 'config/imu_driver.yaml')

# =========================================================== Definitions ========================================================== #

# ----------------------------------------------------------------------------------
# @brief Creates dictionary of arguments for launch_ros.actions.Node action
#    for IMU driver loader
# ----------------------------------------------------------------------------------
def loader_description(fqn_ethercat_node_name, service_wait_timeout, log_level):
    return {

        # Controller node
        'package': 'velmwheel_ethercat_driver',
        'executable': 'driver_loader.py',
        'name': 'imu_driver_loader',
        # Robot's namespace node
        'namespace': f'/{ROBOT_NAME}',
        # Output config
        'output': 'both',
        'emulate_tty': True,
        # Node's parameters
        'parameters': [

            # Driver specification
            { 'plugin_name': plugin_name },
            # Service appearance timeout [s]
            { 'service_wait_timeout': service_wait_timeout },

        ],

        # Node's log level
        'arguments': [ fqn_ethercat_node_name, '--ros-args', '--log-level', log_level ]
        
    }  

# ================================================================================================================================== #
