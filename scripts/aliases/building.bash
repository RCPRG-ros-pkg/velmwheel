# ====================================================================================================================================
# @file       building.bash
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Tuesday, 1st March 2022 4:11:56 pm
# @modified   Monday, 11th July 2022 10:50:35 pm
# @project    engineering-thesis
# @brief      Set of handy function and aliases used when dealing with the project
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ---------------------------------------------------------------------------------------
# @brief Build packages in the src directory and sources setup.bash
# ---------------------------------------------------------------------------------------
function colbuild_base() {

    # Arguments
    local COLCON_SOURCE_DIR=${1:-}
    local CMAKE_FLAGS=${2:-}

    local COLCON_FLAGS=""
    local COLCON_BUILD_FLAGS=""

    enable_word_splitting_locally

    # Add colcon flags
    COLCON_FLAGS+="--log-base ${PROJECT_HOME}/log/build "
    
    # Get list of local packages
    local packages_list=( $(colcon $COLCON_FLAGS list --base-paths $COLCON_SOURCE_DIR | awk '{print $1}') )

    # Add CMake flags
    is_var_set_non_empty CMAKE_FLAGS &&
        COLCON_BUILD_FLAGS+="--cmake-args $CMAKE_FLAGS "
    # Let's allow local packegs' overwitting
    COLCON_BUILD_FLAGS+="--allow-overriding ${packages_list[@]}"

    # Jump to the root directory
    pushd $PROJECT_HOME > /dev/null

    # Build packages
    colbuild -v ${@:3} &&
    # Source install directory
    source $PROJECT_HOME/install/setup.bash || {
    # Handle error
        popd > /dev/null
        return 1    
    }

    # Back to the caller's directory
    popd > /dev/null
}

# ---------------------------------------------------------------------------------------
# @brief Build packages in the src directory and sources setup.bash
# 
# @param ... 
#    parameters passed to the `colbuild` command
# ---------------------------------------------------------------------------------------
function colbuild_src() {

    log_info "Building source directory..."

    # Prepare source paths
    local SOURCE_DIRS="$PROJECT_HOME/src"
    # Build
    colbuild_base "$SOURCE_DIRS" "" "$@" || {
    # On error return
        log_error "Failed to build source directory"
        return 1
    }

    log_info "Source directory built"
}

# ---------------------------------------------------------------------------------------
# @brief Build packages in the source extern directories
# 
# @param ... 
#    parameters passed to the `colbuild` command
# ---------------------------------------------------------------------------------------
function colbuild_extern() {

    log_info "Building extern/ros directory..."

    # Prepare source paths
    local SOURCE_DIRS="$PROJECT_HOME/extern/ros"
    # Prepare CMake flags paths (disable LDMRS Sick LIDARs support in the producer's ROS package)
    local CMAKE_FLAGS="-DBUILD_WITH_LDMRS_SUPPORT=OFF"
    # Build external dependencies
    colbuild_base "$SOURCE_DIRS" "$CMAKE_FLAGS" "$@" || {
    # On error return
        log_error "Failed to build extern directory"
        return 1
    }

    log_info "Extern directory built"
}

# ---------------------------------------------------------------------------------------
# @brief Build packages in the source directories and extern directories
# 
# @param ... 
#    parameters passed to the `colbuild` command
# ---------------------------------------------------------------------------------------
function colbuild_prj() {
    colbuild_extern "$@"
    colbuild_src "$@"
}

# ---------------------------------------------------------------------------------------
# @brief Build packages required by the simulation environment
# @depreciated
# ---------------------------------------------------------------------------------------
function colbuild_sim_packages() {

    # Build packages
    COLCON_SOURCE_DIR="$PROJECT_HOME/src" \
    colbuild -v                           \
        'regex_forwarder'                 \
        'node_common'                     \
        'velmwheel_msgs'                  \
        'velmwheel_model'                 \
        'velmwheel_launch'                &&
        'velmwheel_gazebo'                \
    # Source install directory
    source $PROJECT_HOME/install/setup.bash
}


# ---------------------------------------------------------------------------------------
# @brief Build required packages and runs robot's simulation in the default configuration
# @depreciated
# ---------------------------------------------------------------------------------------
function colbuild_and_run_default_sim() {

    # Kill all previous packages
    kill_system &&
    # Build packages
    colbuild_sim_packages &&
    # Run simulation
    run_default_sim $@
}


# ---------------------------------------------------------------------------------------
# @brief Builds doxygen documentation for the project
# @depreciated
# ---------------------------------------------------------------------------------------
function build_doxygen_doc() {

    # Create log directory
    mkdir -p $PROJECT_HOME/log/doc
    # Create log file
    touch $PROJECT_HOME/log/doc/doxygen.log
    # Run doxygen
    doxygen Doxyfile
}


# ---------------------------------------------------------------------------------------
# @brief Deletes __pycache__ directories from src/ and extern/ subdirectories
# ---------------------------------------------------------------------------------------
function clear_pycaches() {
    find ./extern ./src -name *__pycache__* -exec rm -rf {} +
}
