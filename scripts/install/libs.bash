# ====================================================================================================================================
# @file       libs.bash
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Wednesday, 20th April 2022 11:26:33 pm
# @modified   Wednesday, 20th April 2022 11:46:59 pm
# @project    Winder
# @brief
#    
#    Script installs extarnal libraries required by the project
#    
# @copyright Krzysztof Pierczyk © 2021
# ====================================================================================================================================

# Source bash-utils library
source $BASH_UTILS_HOME/source_me.bash

# ============================================================ Functions =========================================================== #

# -----------------------------------------------------------------------------
# @brief Build extern library named @p lib_name 
# -----------------------------------------------------------------------------
function build_extern_library() {

    # Argumenrs
    local lib_name="$1"
    local cmake_args=( "${@:2}" )

    local LIB_SRC_DIR="${PROJECT_HOME}/extern/$lib_name"
    local LIB_BUILD_DIR="${PROJECT_HOME}/build/$lib_name"
    local LIB_INSTALL_DIR="${PROJECT_HOME}/install/$lib_name"

    # Install library (if not built)
    if ! [[ -d "${LIB_INSTALL_DIR}" ]] || ! [[ -n "$(ls -A $LIB_INSTALL_DIR)" ]]; then
        
        log_info "Installing $lib_name library..."

        # Remove build directory
        rm -rf "${LIB_BUILD_DIR}"
        # Create build directory
        mkdir -p "${LIB_BUILD_DIR}"
        # Enter build directory
        pushd ${LIB_BUILD_DIR} > /dev/null
        # Configure CMake
        cmake -DCMAKE_INSTALL_PREFIX=${LIB_INSTALL_DIR} "${cmake_args[@]}" ${LIB_SRC_DIR} || {
            popd > /dev/null;
            log_error "Failed to configure $lib_name library"
            return 1
        }

        log_info "$lib_name library configured"

        # Build library
        make install || {
            popd > /dev/null;
            log_error "Failed to build $lib_name library"
            return 1
        } 

        log_info "$lib_name library installed"
        
        popd > /dev/null;
    fi

}

# ============================================================== Main ============================================================== #

main() {

    # Build cpp-utils
    build_extern_library "cpp-utils"

    # Build itsy-bitsy
    build_extern_library "itsy_bitsy" \
        "-DBUILD_TESTING=OFF"         \
        "-DITSY_BITSY_SINGLE=OFF"
        
}

# ============================================================= Script ============================================================= #

# Run the script
source $BASH_UTILS_HOME/lib/scripting/templates/base.bash
