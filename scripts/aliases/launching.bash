# ====================================================================================================================================
# @file       launching.bash
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Tuesday, 1st March 2022 4:11:56 pm
# @modified   Monday, 18th July 2022 8:28:13 pm
# @project    engineering-thesis
# @brief      Set of handy function and aliases used when dealing with the project
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ---------------------------------------------------------------------------------------
# @brief Kills all processes that could be left open after previous launch
# ---------------------------------------------------------------------------------------
function kill_system() {

    # Kill gazebo client/srver
    pkill -9 gzserver
    pkill -9 gzclient
    
    # Kill all related processes
    pkill -9 -f ".*velmwheel_.*"
    pkill -9 -f ".*/velmwheel/install/*"
    pkill -9 -f "/opt/ros/humble/*"

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

    # Configure launch system to run simulated version of low-level drivers
    launch_params+=( launch_mode:=sim )
    # Configure GUI and RVIZ to be run
    launch_params+=( with_gui:=true  )
    launch_params+=( with_rviz:=true )
    # Configure publishing mode of LIDAR drivers
    launch_params+=( laser_output_mode:=both )
    # Configure static transformation for key frames
    launch_params+=( map_at:=world       )
    # launch_params+=( odom_at:=map        )
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

# ---------------------------------------------------------------------------------------
# @brief Runs Velmwheel system 
#
# @param ...
#    other parameters to be passed to `ros2 launch` command
# ---------------------------------------------------------------------------------------
function bringup() {

    # Arguments
    local world="$1"

    # On success, clear terminal and kill all running components of the system
    clear && kill_system || return 1

    local -a launch_params=()

    # Configure launch system to run simulated version of low-level drivers
    launch_params+=( launch_mode:=real )
    # Configure log level
    launch_params+=( velmwheel_log_level:=info )
    # Configure RVIZ to be run
    launch_params+=( with_rviz:=false )

    # Configure laser drivers
    launch_params+=( with_lidar_l:=true )
    launch_params+=( with_lidar_r:=true )
    # Configure EtherCAT drivers
    launch_params+=( with_ethercat_driver:=true )

    # Configure publishing mode of LIDAR drivers
    launch_params+=( laser_output_mode:=both )
    # Configure static transformation for key frames
    launch_params+=( map_at:=world       )
    # launch_params+=( odom_at:=map        )
    launch_params+=( robot_at:=odom_pose )
    # Configure components to be run/not-run
    launch_params+=( with_base_controller:=true      )
    launch_params+=( with_laser_conversion:=true     )
    launch_params+=( with_laser_odom:=true           )
    launch_params+=( with_odom_fusion:=true          )
    launch_params+=( with_bias_estimator:=true       )
    launch_params+=( with_poi_map_builder:=false     )
    launch_params+=( with_global_localization:=false )
    launch_params+=( with_slam:=true                )
    
    # Run simulation
    ros2 launch velmwheel_bringup launch.py "${launch_params[@]}" "${@:2}"

}
