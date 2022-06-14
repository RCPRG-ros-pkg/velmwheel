# ====================================================================================================================================
# @file       ethercat_driver.launch.py
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Thursday, 7th April 2022 6:50:10 am
# @modified   Tuesday, 14th June 2022 4:34:37 pm
# @project    engineering-thesis
# @brief      Launchfile running middleware of the WUT Velmwheel robot's `ethercat_driver` module
#    
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Imports ============================================================ #

# Launch imports
from launch import LaunchDescription, LaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch_ros.actions import Node
# Private imports
from launch_common.config import get_config_source
from launch_common.arguments import declare_launch_argument
import velmwheel_ethercat_driver.launch.ethercat_driver as ethercat_driver
import velmwheel_base_driver.launch.description as base_driver
import velmwheel_imu_driver.launch.description as imu_driver

# ============================================================ Arguments =========================================================== #

# Launch argument: log level of all nodes
declare_velmwheel_log_level_description, velmwheel_log_level_config = declare_launch_argument({
    'name':          'velmwheel_log_level',
    'default_value': 'warn',
    'description':   'Log level of the run nodes'
})

# =========================================================== Bus driver =========================================================== #

# EtherCAT bus driver node
bus_driver_description = {

    # Source file
    'launch_description_source': LaunchDescriptionSource(launch_description=ethercat_driver.generate_launch_description(

        # Fixed-defined carameters
        params = [

            # Drivers to be loaded
            { 'initial_drivers': [ 
                base_driver.plugin_name,
                imu_driver.plugin_name 
            ] },

            # Configuration file (base driver)
            get_config_source('velmwheel_drivers_bringup', 'base_driver.yaml'),
            # Configuration file (IMU driver)
            get_config_source('velmwheel_drivers_bringup', 'imu_driver.yaml'),

        ],

    )),
    # Arguments
    'launch_arguments': {

        # Configuration file (bus driver)
        f'ethercat_driver_config': get_config_source('velmwheel_drivers_bringup', 'ethercat_driver.yaml')
        
    }.items()
}

# =========================================================== Description ========================================================== #

# Description of the launch
launch_description = [
    
    # Arguments
    DeclareLaunchArgument(**declare_velmwheel_log_level_description),
    # Includes
    IncludeLaunchDescription(**bus_driver_description),

]

# ================================================================================================================================== #

def generate_launch_description():
    return LaunchDescription(launch_description)
