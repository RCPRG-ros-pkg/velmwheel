# ====================================================================================================================================
# @file       launch.py
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Thursday, 7th April 2022 6:50:10 am
# @modified   Wednesday, 25th May 2022 4:44:01 pm
# @project    engineering-thesis
# @brief      Main launch file for the WUT Velmwheel robot
#    
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Imports ============================================================ #

# System imports
from os import path
# Launch imports
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from launch_common.arguments import declare_launch_argument
# Private imports
from launch_common.config import get_config_source
from launch_common.includes import get_launch_source

# =========================================================== Arguments ============================================================ #

# Launch argument: launchmode of the robot 
declare_launch_mode_config_description, launch_mode_config = declare_launch_argument({
    'name':          'launch_mode',
    'default_value': 'sim',
    'description':   'Launchmode of the robot (either "sim" or "real")'
})

# ============================================================= Helpers ============================================================ #

# ROS-Launch condition evaluating to True at run-time if 'sim' run is performed
sim_run = IfCondition(PythonExpression([ '"', launch_mode_config, f'" == "sim"' ]))
# ROS-Launch condition evaluating to True at run-time if 'real' run is performed
real_run = IfCondition(PythonExpression([ '"', launch_mode_config, f'" == "real"' ]))

# ============================================================ Low-level =========================================================== #

# Gazbeo simulation
sim = {

    # Source file
    'launch_description_source': get_launch_source('velmwheel_sim_bringup', 'gazebo.launch.py'),
    # Conditionally (if sim-based launch is run)
    'condition': sim_run
    
}

# Low-level drivers
drivers = {

    # Source file
    'launch_description_source': get_launch_source('velmwheel_drivers_bringup', 'drivers.launch.py'),
    # Conditionally (if sim-based launch is run)
    'condition': real_run
    
}

# =========================================================== Middleware =========================================================== #

# Middleware software description
def middleware_description(simulation_launch):
    return {

        # Source file
        'launch_description_source': get_launch_source('velmwheel_middleware_bringup', 'middleware.launch.py'),

        # Condition
        'condition': sim_run if simulation_launch else real_run,

        # Parameters
        'launch_arguments': {
            'use_sim_time': 'true' if simulation_launch else 'false'
        }.items()

    }

# Middleware software (simulation)
middleware_sim = middleware_description( simulation_launch=True )
# Middleware software (non-simulation)
middleware_real = middleware_description( simulation_launch=False )

# ============================================================ Utilities =========================================================== #

# RVIZ utility
rviz = { 

    # Source
    'launch_description_source': get_launch_source('velmwheel_launch', 'rviz.launch.py'),
    
    # Parameters
    'launch_arguments': {
        'rviz_config': get_config_source('velmwheel_bringup', 'velmwheel.rviz')
    }.items()
    
}

# Standard-frames transform publishers
map_transform_publisher   = { 'launch_description_source': get_launch_source('velmwheel_bringup', 'helpers/map_at.launch.py'   ) }
#odom_transform_publisher  = { 'launch_description_source': get_launch_source('velmwheel_bringup', 'helpers/odom_at.launch.py'  ) }
robot_transform_publisher = { 'launch_description_source': get_launch_source('velmwheel_bringup', 'helpers/robot_at.launch.py' ) }

# =========================================================== Description ========================================================== #

# Description of the launch
launch_description = [
    
    # Arguments
    DeclareLaunchArgument(**declare_launch_mode_config_description),

    # Low-level
    IncludeLaunchDescription(**sim),
    IncludeLaunchDescription(**drivers),
    # Middleware software
    IncludeLaunchDescription(**middleware_sim),
    IncludeLaunchDescription(**middleware_real),
    # Utilities
    IncludeLaunchDescription(**rviz),
    IncludeLaunchDescription(**map_transform_publisher),
    IncludeLaunchDescription(**odom_transform_publisher),
    IncludeLaunchDescription(**robot_transform_publisher),

]

# ================================================================================================================================== #

def generate_launch_description():
    return LaunchDescription(launch_description)
