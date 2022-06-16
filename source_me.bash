# ====================================================================================================================================
# @file       sourceMe.bash
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Friday, 23rd April 2021 12:29:15 pm
# @modified   Wednesday, 15th June 2022 2:38:45 pm
# @project    engineering-thesis
# @brief      Script that should be sourced before starting working with the directory
# @details
#   
#    Script provides user's terminal with definitions required for project's handling. Morover, it checks for project's dependancies
#    and installs them, if needed.
#
#    If the 'update' argument is passed to the script, the `git submodule update --init --recursive` will be run and all system
#    dependencies will be verified and installed as needed. This is optional action as veifying all dependencies may take  a longer 
#    while and it makes no sense to repeat this process at each source action. It is required to run update command at least once 
#    after cloning the repository.
#
#    Optionally, one may source this script with the 'setup' keyword which will make the script verify dependencies without
#    updating git submodules. This may be handy if some system dependencies are added to the scripts/isntall/*.bash scripts
#    and need to be installed.
#
# @note Script was tested on Ubuntu 20.0.4 LTS system
# @note This script should be rather sourced than executed
#
# @copyright Krzysztof Pierczyk © 2021
# ====================================================================================================================================

# ========================================================== Configruation ========================================================= #

# Set to the project's home directory
export PROJECT_HOME="$(dirname "$(readlink -f "$BASH_SOURCE")")"

# Set distro of the ROS2 to be used
export PROJECT_ROS_DISTRO=galactic
# Domain ID for ROS2 DDS middleware
export ROS_DOMAIN_ID=0
# ROS home and logs [default values at the moment]
export ROS_HOME="$HOME/.ros"
export ROS_LOG_DIR="$PROJECT_HOME/log/launch"

# Name of the default docker image to be run with commands
export DFLT_DOCKER_IMG=ros:latest

# ============================================================= Helpers ============================================================ #

# Auxiliary log_info used before sourceing `bash-utils`
function log_info_aux()  { echo -e "[\033[32mINFO\033[0m]"  "$@"; }
function log_error_aux() { echo -e "[\033[31mERROR\033[0m]" "$@"; }

# ========================================================== Dependencies ========================================================== #

# Helper function setuping project's system
function setup_project() {

    log_info_aux "Resolving 'bash-utils' dependencies..."

    # Source bash-utils library
    source $PROJECT_HOME/extern/bash-utils/source_me.bash setup || {
        log_error_aux "Failed to resolve 'bash-utils' dependencies"
        return 1
    }
    
    # Configure environment before dependencies' installation
    source $PROJECT_HOME/scripts/preconfig.bash || return 1

    log_info "Installing dependencies..."

    # Install dependencies
    $PROJECT_HOME/scripts/install.bash || {
        log_error "Failed to install dependencies"
        return 1
    }

    log_info "Applying patched dependencies..."

    # Apply patches
    $PROJECT_HOME/scripts/apply_patches.bash || {
        log_error "Failed to apply required patches"
        return 1
    }

}

# Update project
if [[ "$1" == 'update' ]]; then

    log_info_aux "Clonning git submodules..."

    # Clone submodules
    git submodule update --init --recursive || {
        log_error_aux "Failed to clone git submodules"
        return 1
    }

    # Setup project 
    setup_project || return 1
    
# Setup project
elif [[ "$1" == 'setup' ]]; then

    # Setup project 
    setup_project || return 1
    
# Just prepare current shell
else

    # Source bash-utils library
    source $PROJECT_HOME/extern/bash-utils/source_me.bash || return 1
    # Configure environment before dependencies' installation
    source $PROJECT_HOME/scripts/preconfig.bash || return 1

fi

# Unset temporary functions
unset -f log_info_aux
unset -f log_error_aux
unset -f setup_project

# ========================================================== Configuration ========================================================= #

# Configure dependencies
source $PROJECT_HOME/scripts/config.bash || {
    log_error "Failed to configure environment"
    return 1
}

# Source the ROS2 workspace if built
if [[ -e $PROJECT_HOME/install/setup.bash ]]; then
    source $PROJECT_HOME/install/setup.bash
fi

# Source helper aliases
source $PROJECT_HOME/scripts/aliases.bash
