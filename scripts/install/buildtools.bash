# ====================================================================================================================================
# @file       buildtools.bash
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Tuesday, 26th October 2021 3:08:37 pm
# @modified   Wednesday, 15th June 2022 11:22:09 pm
# @project    Winder
# @brief      Script checks building-tools dependencies and installs them if required
#    
#    
# @copyright Krzysztof Pierczyk © 2021
# ====================================================================================================================================

# Source bash-utils library
source $BASH_UTILS_HOME/source_me.bash

# ============================================================== Main ============================================================== #

main() {
    
    # Install colcon
    if ! is_pkg_installed python3-colcon-common-extensions; then
        $BASH_UTILS_BIN_HOME/install/buildtools/colcon.bash
    fi

}

# ============================================================= Script ============================================================= #

# Run the script
source $BASH_UTILS_HOME/lib/scripting/templates/base.bash
