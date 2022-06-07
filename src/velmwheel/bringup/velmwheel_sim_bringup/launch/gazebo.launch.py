# ====================================================================================================================================
# @file       gazebo.launch.py
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Sunday, 6th March 2022 7:04:33 pm
# @modified   Wednesday, 25th May 2022 11:19:47 pm
# @project    engineering-thesis
# @brief      Launchfile running WUT Velmwheel simulation in the Gazebo simulator along with the robot state and joints publishers
# @details 
#    
#    Launchfile running WUT Velmwheel simulation in the Gazebo simulator. User may choose between URDF- and SDF-based 
#    description of the robot. At the moment the SDF-based description is not recommended. Due to lack of parametrized includes 
#    in libsdformat9 there is no way for parametrizing submodels of robot's LIDAR sensors resulting in both LIDAR pluging publishing
#    sensorical data to the same topic.
# 
#    The temporary solution could be to provide separated, boilerplate definitions for both sensors but it would an ugly solution). 
#    Instead, Velmwheel simulation has been switched to the URDF description. This one has an advantage of being the native format
#    for the `rviz2`. Unfortunately, the Gazbo-built-in URDF-SDF converter arrives with some bugd in Gazebo-11 that make produced
#    modols looking bad (depending on the setup either in the `rviz` or `gazebo`). To solve this problem, the underlying launch 
#    provides a simple rgex-based fine-tunning mechanisms that fies some major issues with the robot's description published
#    on the '/robot_description' topic. Such a modified version of the URDF files is then passed to the '/spawn_entity' service
#    as a base for genereting the SDF description for Gazebo. This mechanism introduces very limited complexity into the project 
#    and provides a highly interface for using native robot description formats in both `rviz` and `gazebo`.
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Imports ============================================================ #

# Launch imports
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
# Private imports
from launch_common.includes import get_launch_source
from launch_common.models import get_urdf_source
    
# ============================================================= Gazebo ============================================================= #

# Gazbeo simulation
gazebo = {

    # Source file
    'launch_description_source': get_launch_source('velmwheel_gazebo', 'gazebo.launch.py')
}

# ======================================================= Robot-related nodes ====================================================== #

# Robot spawning node
robot_spawner = {

    # Source file
    'launch_description_source': get_launch_source('velmwheel_gazebo', 'robot_spawner.launch.py')
}

# Robot state and joints publisher
state_publisher = {

    # Source file
    'launch_description_source': get_launch_source('velmwheel_launch', 'state_publisher.launch.py'),

    # Launch Parameters
    'launch_arguments': {
        'urdf_path': get_urdf_source('velmwheel_gazebo', 'velmwheel.urdf.xacro'),
        'use_sim_time': 'true',
    }.items(),
    
}

# =========================================================== Description ========================================================== #

# Description of the launch
launch_description = [
    
    # Gazebo
    IncludeLaunchDescription(**gazebo),
    # Robot-related
    IncludeLaunchDescription(**robot_spawner),
    IncludeLaunchDescription(**state_publisher),

]

# ================================================================================================================================== #

def generate_launch_description():
    return LaunchDescription(launch_description)
