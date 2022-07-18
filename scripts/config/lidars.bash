# ====================================================================================================================================
# @file       lidars.bash
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Friday, 24th June 2022 12:29:07 pm
# @modified   Thursday, 7th July 2022 7:35:23 pm
# @project    engineering-thesis
# @brief      Script configuring net hardware for communication with LIDAR sensors of the WUT Velmwheel robot
# 
# 
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# Source bash-utils library
source $BASH_UTILS_HOME/source_me.bash

# ============================================================= Script ============================================================= #

# Script's log context
LOG_CONTEXT="lidars-config"

# ============================================================== Main ============================================================== #

main() {
    
    # Enable NTP server for LIDARs
    sudo systemctl start ntp

    # If vali interface has been choose
    if [[ "$LIDAR_INTERFACE_NAME" != "" ]]; then

        # Check if interface exsists
        if ifconfig "$LIDAR_INTERFACE_NAME" &> /dev/null; then

            # Enable the interface
            sudo ifconfig "$LIDAR_INTERFACE_NAME" up &> /dev/null || {
                log_error "Failed to enable '$LIDAR_INTERFACE_NAME' net interface for communication with LIDAR sensors"
                return 1
            }

            # Check if IP needs to be confiugured manually
            if [[ "$LIDAR_INTERFACE_IP" != "" ]]; then

                # Try to configure interface's IP
                sudo ifconfig "$LIDAR_INTERFACE_NAME" "$LIDAR_INTERFACE_IP" || {
                    log_error "Failed to configure IP of the '$LIDAR_INTERFACE_NAME' net interface to '$LIDAR_INTERFACE_IP'"
                    return 1
                }

            fi

        # If interface does not exist, consider it unused and exit
        else
            return 0
        fi

    fi

}

# ============================================================= Script ============================================================= #

# Run the script
source $BASH_UTILS_HOME/lib/scripting/templates/base.bash
