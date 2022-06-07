# ====================================================================================================================================
# @file     launching.bash
# @author   Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date     Tuesday, 1st March 2022 4:11:56 pm
# @modified   Friday, 20th May 2022 11:34:47 am
# @project  engineering-thesis
# @brief
#    
#    Set of handy function and aliases used when dealing with the project
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ---------------------------------------------------------------------------------------
# @brief Kills all processes that could be left open after previous launch
# ---------------------------------------------------------------------------------------
function kill_system() {

    # Kill gazebo client/srver
    pkill gzserver
    pkill gzclient
    
    # Kill all related processes
    pkill -f ".*velmwheel_.*"

    return 0
}

# ---------------------------------------------------------------------------------------
# @brief Runs Gazbeo simulation with the given world
#
# @param world
#    world to be run (relative to the gazebo/media/worlds directory of the gazebo
#    simulation package)
# @param ...
#    other parameters to be passed to `ros2 launch` command
# ---------------------------------------------------------------------------------------
function bringup_sim() {

    # Arguments
    local world="$1"

    # On success, clear terminal and kill all running components of the system
    clear && kill_system || return 1

    local -a launch_params=()

    # Configure GUI and RVIZ to be run
    launch_params+=( with_gui:=true  )
    launch_params+=( with_rviz:=true )
    # Configure publishing mode of LIDAR drivers
    launch_params+=( laser_output_mode:=both )
    # Configure static transformation for key frames
    launch_params+=( map_at:=world       )
    launch_params+=( odom_at:=map        )
    launch_params+=( robot_at:=odom_pose )
    # Configure components to be run/not-run
    launch_params+=( with_base_controller:=true      )
    launch_params+=( with_laser_conversion:=true     )
    launch_params+=( with_laser_odom:=true           )
    launch_params+=( with_odom_fusion:=true          )
    launch_params+=( with_bias_estimator:=true       )
    launch_params+=( with_poi_map_builder:=false     )
    launch_params+=( with_global_localization:=false )
    launch_params+=( with_slam:=true                 )
    # Configure world to be run
    launch_params+=( sim_world:="$world" )
    
    # Run simulation
    ros2 launch velmwheel_bringup launch.py "${launch_params[@]}" "${@:2}"

}

# ---------------------------------------------------------------------------------------
# @brief Buids all source packages and runs Gazbeo simulation with the given world
#
# @param world
#    world to be run (relative to the gazebo/media/worlds directory of the gazebo
#    simulation package)
# @param ...
#    other parameters to be passed to `ros2 launch` command
# ---------------------------------------------------------------------------------------
function build_and_bringup_sim() {

    # Arguments
    local world="$1"

    # Build source packages
    colbuild_src || return 1
    # On success, run simulation
    bringup_sim "$world" "${@:2}"
}
