# ====================================================================================================================================
# @file       apply_patches.bash
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Thursday, 26th May 2022 3:21:59 am
# @modified   Friday, 24th June 2022 4:02:45 pm
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

    # --------------------------- Apply [sick_scan_xd] patches ------------------------ #
    
    # Path to the patch file
    local PATCH_FILE=$PROJECT_HOME/scripts/patches/sick_scan_xd/CMakeLists.txt.patch
    # Path to the patched file
    local PATCHED_FILE=$PROJECT_HOME/extern/ros/sick_scan_xd/CMakeLists.txt

    local SICK_SCAN_XD_PATCHED=0

    # Patch the file
    if ! patch -R -p0 -s -f --dry-run "$PATCHED_FILE" "$PATCH_FILE" > /dev/null; then
        log_info "Patching CMakeLists.txt file from [sick_scan_xd] package..."
        patch -p0 "$PATCHED_FILE" "$PATCH_FILE" >/dev/null || {
            log_warning "Failed to patch [sick_scan_xd] package."
        }
        SICK_SCAN_XD_PATCHED=1
    fi
    
    # Path to the patch file
    local PATCH_FILE=$PROJECT_HOME/scripts/patches/sick_scan_xd/sick_scan_common.cpp.patch
    # Path to the patched file
    local PATCHED_FILE=$PROJECT_HOME/extern/ros/sick_scan_xd/driver/src/sick_scan_common.cpp

    # Patch the file
    if ! patch -R -p0 -s -f --dry-run "$PATCHED_FILE" "$PATCH_FILE" > /dev/null; then
        log_info "Patching sick_scan_common.cpp file from [sick_scan_xd] package..."
        patch -p0 "$PATCHED_FILE" "$PATCH_FILE" >/dev/null || {
            log_warning "Failed to patch [sick_scan_xd] package."
        }
        SICK_SCAN_XD_PATCHED=1
    fi
    
    # Path to the patch file
    local PATCH_FILE=$PROJECT_HOME/scripts/patches/sick_scan_xd/sick_scan_marker.cpp.patch
    # Path to the patched file
    local PATCHED_FILE=$PROJECT_HOME/extern/ros/sick_scan_xd/driver/src/sick_scan_marker.cpp

    # Patch the file
    if ! patch -R -p0 -s -f --dry-run "$PATCHED_FILE" "$PATCH_FILE" > /dev/null; then
        log_info "Patching sick_scan_marker.cpp file from [sick_scan_xd] package..."
        patch -p0 "$PATCHED_FILE" "$PATCH_FILE" >/dev/null || {
            log_warning "Failed to patch [sick_scan_xd] package."
        }
        SICK_SCAN_XD_PATCHED=1
    fi

    # Log after-patch info
    if [[ $SICK_SCAN_XD_PATCHED == "1" ]]; then
        log_info "[sick_scan_xd] package patched"
    fi

}

# ============================================================= Script ============================================================= #

# Run the script
source $BASH_UTILS_HOME/lib/scripting/templates/base.bash