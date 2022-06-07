# ====================================================================================================================================
# @file     simulation.bash
# @author   Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date     Tuesday, 1st March 2022 4:11:56 pm
# @modified   Monday, 25th April 2022 2:52:16 pm
# @project  engineering-thesis
# @brief
#    
#    Set of handy function and aliases used when dealing with the project
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ---------------------------------------------------------------------------------------
# @brief Runs robot's simulation
#
# @options
#
#                  --gui  runs simulation in the GUI mode (gzclient)
#                 --rviz  runs simulation with the rviz
#             -i|--ideal  runs idealize robot's driver (by default uses real on)
#             --sim-pose  runs additional node publishing real position of the robot
#                         in the simulation
#    -l|--laser-out-mode  output mode of the LIDAR nodes to be run (either 'separate', 
#                         'common' or 'both')
#
# @depreciated
# ---------------------------------------------------------------------------------------
function run_sim() {

    # ---------------- Parse arguments ----------------

    # Function's options
    declare -a opt_definitions=(
        '--gui',gui,f
        '--rviz',rviz,f
        '-i|--ideal',ideal,f
        '--sim-pose',sim_pose,f
        '-l|--laser-out-mode',laser_out_mode
    )

    # Parse arguments to a named array
    parse_options_s

    # Set list of packages as positional arguments
    set -- "${posargs[@]}"   

    # ---------------- Parse arguments ----------------
    
    # Kill the system 
    kill_system

    # Add GUI argument, if requested
    local gui_flags_=""
    is_var_set options[gui] &&
        gui_flags_="with_gui:=true"
    # Add RVIZ argument, if requested
    local rviz_flags_=""
    is_var_set options[rviz] &&
        rviz_flags_="with_rviz:=true"
    # Select driver mode
    local driver_mode_="base_plugin:=real"
    is_var_set options[ideal] &&
        driver_mode_="base_plugin:=ideal"
    # Select driver mode
    local sim_pose_=""
    is_var_set options[sim_pose] &&
        sim_pose_="with_simulation_pose_pub:=true"
    # Select LIADR scans output
    local laser_output_mode_=""
    is_var_set options[laser_out_mode] &&
        laser_output_mode_="laser_output_mode:=${options[laser_out_mode]}"
        
    # Source install directory
    source $PROJECT_HOME/install/setup.bash
    
    # Run the simulation
    ros2 launch velmwheel_gazebo sim.launch.py \
        $gui_flags_                            \
        $rviz_flags_                           \
        $driver_mode_                          \
        $sim_pose_                             \
        $laser_output_mode_                    \
        $@
}

# ---------------------------------------------------------------------------------------
# @brief Build required packages and runs robot's simulation
# @depreciated
# ---------------------------------------------------------------------------------------
function build_and_run_sim() {

    # Kill all previous packages
    kill_system &&
    # Build packages
    colbuild_sim_packages &&
    # Run simulation
    run_sim $@
}
