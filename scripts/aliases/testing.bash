# ====================================================================================================================================
# @file       testing.bash
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Tuesday, 1st March 2022 4:11:56 pm
# @modified   Monday, 11th July 2022 10:35:07 pm
# @project    engineering-thesis
# @brief      Set of handy function and aliases used when dealing with the project
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ---------------------------------------------------------------------------------------
# @brief Tests packages in the src directory and sources setup.bash
# ---------------------------------------------------------------------------------------
function coltest_src() {

    # Set source directory
    local COLCON_SOURCE_DIR=$PROJECT_HOME/src

    # Add colcon flags
    local COLCON_FLAGS="--log-base ${PROJECT_HOME}/log/build "
    
    # Get list of local packages
    local packages_list=( $(colcon $COLCON_FLAGS list --base-paths $COLCON_SOURCE_DIR | awk '{print $1}') )
    # Let's allow local packegs' overwitting
    COLCON_TEST_FLAGS="--allow-overriding ${packages_list[@]}"

    # Test packages
    coltest -v --fv "$@"
}
