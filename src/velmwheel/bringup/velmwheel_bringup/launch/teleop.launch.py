# ====================================================================================================================================
# @file       teleop.launch.py
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Monday, 11th April 2022 6:13:33 pm
# @modified   Wednesday, 25th May 2022 4:44:01 pm
# @project    engineering-thesis
# @brief      Launch running the 'teleop_twist_keyboard' node preconfigured for driving WUT Velmwheel robot
#    
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Imports ============================================================ #

# Launch imports
from launch import LaunchDescription
from launch_ros.actions import Node
# Private imports
from velmwheel_model.params import ROBOT_NAME

# ========================================================= Node descriptor ======================================================== #

# Teleop node
teleop = {

    # Executable
    'package': 'teleop_twist_keyboard',
    'executable': 'teleop_twist_keyboard',
    'name': 'keyboard_teleop',
    # Namespace
    'namespace': f'/{ROBOT_NAME}',
    # Output configuration
    'output': 'both',
    'prefix': 'xterm -e',
    
    # Remap I/O topics
    'remappings': [

        # Outputs
        ( 'cmd_vel', f'/{ROBOT_NAME}/base/velocity_setpoint' ),

    ],
    
}

# =========================================================== Description ========================================================== #

# Description of the launch
launch_description = [
    
    # Nodes
    Node(**teleop),

]

# ================================================================================================================================== #

def generate_launch_description():
    return LaunchDescription(launch_description)

