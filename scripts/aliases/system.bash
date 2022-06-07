# ====================================================================================================================================
# @file     system.bash
# @author   Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date     Tuesday, 1st March 2022 4:11:56 pm
# @modified   Thursday, 7th April 2022 5:39:27 pm
# @project  engineering-thesis
# @brief
#    
#    Set of handy function and aliases used when dealing with the project
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ---------------------------------------------------------------------------------------
# @brief Runs robot's base controller
#
# @options
#
#      -o|--odom-at-world  runs static transform publisher placing "odom" frame at "world"
#
# ---------------------------------------------------------------------------------------
function run_base_control() {

    # ---------------- Parse arguments ----------------

    # Function's options
    declare -a opt_definitions=(
        '-o|--odom-at-world',odom_at_world,f
        '-s|--use-sim-time',use_sim_time,f
    )

    # Parse arguments to a named array
    parse_options_s

    # Set list of packages as positional arguments
    set -- "${posargs[@]}"   

    # ---------------- Parse arguments ----------------
    
    local flags_=""

    # Check if 'odom' frame has to be placed at 'world'
    is_var_set options[odom_at_world] &&
        flags_+="odom_at_world:=true "
    # Check if simulated time is to be used
    is_var_set options[use_sim_time] &&
        flags_+="use_sim_time:=true "
    
    # Run the node
    ros2 launch velmwheel_base_controller \
        base_controller.launch.py         \
        $flags_                           \
        $@
}
