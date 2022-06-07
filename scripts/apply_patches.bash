# ====================================================================================================================================
# @file       apply_patches.bash
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Thursday, 26th May 2022 3:21:59 am
# @modified   Thursday, 26th May 2022 3:34:00 am
# @project    engineering-thesis
# @brief      Auxiliary script applying patches required by the environment
# 
# 
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# Source bsah-helper library
source $PROJECT_HOME/extern/bash-utils/source_me.bash

# ============================================================ Functions =========================================================== #

main() {

    # ------------------------------ Apply [rclpy] patches ---------------------------- #

    # --------------------------------------------------------------------------
    # @brief According to [1] overriding ros-defined parameter (like use_sim_time)
    #    at launch of the Python node leads to the 'Parameter(s) already declared' 
    #    error. The node.py requires patch to make it able to set 'use_sim_time' 
    #    ROS parameter at node's launch. As this patch has not been introduced 
    #    yet, here is it's local implementation according to [2]
    #
    # @see [1] https://github.com/ros2/rclpy/issues/942
    # @see [2] https://github.com/ros2/rclpy/commit/2c9ad988b277439d6faf41edf1cb28519e025fb5
    # --------------------------------------------------------------------------
    
    # Path to the patch file
    local PATCH_FILE=$PROJECT_HOME/scripts/patches/node.py.patch
    # Path to the patched file
    local PATCHED_FILE=/opt/ros/${ROS_DISTRO}/lib/python3.8/site-packages/rclpy/node.py

    # Patch the file
    if ! patch -R -p0 -s -f --dry-run "$PATCHED_FILE" "$PATCH_FILE" > /dev/null; then
        log_info "Patching node.py file from [rclpy] package..."
        sudo patch -p0 "$PATCHED_FILE" "$PATCH_FILE" || {
        log_error "Failed to patch [rclpy] package"
            return 1
        }
        log_info "[rclpy] package patched"
    fi

}

# ============================================================= Script ============================================================= #

# Run the script
source $BASH_UTILS_HOME/lib/scripting/templates/base.bash