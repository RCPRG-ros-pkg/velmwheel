# ====================================================================================================================================
# @file       cifx.bash
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Thursday, 14th January 2021 9:29:40 pm
# @modified   Monday, 18th July 2022 5:53:37 pm
# @project    engineering-thesis
# @brief      Downloads, builds, installs and loads uio_netx kernel that maps Hilscher card's memory and interrupts into Linux
#             UIO (Userspace I/O) subsystem. It is required by all drivers operating on these cards.
#
# @note UIO NETX V2.0.1.0 has problem building as ioremap_nocache() is no longer present in the Linux'es sources. This
#    script changes all calls to this function into calls to ioremap_cache().
# @note Before running this script `source_me.bash` (under project's home directory) should be sourced by the calling terminal
# @note Script was tested on Ubuntu 22.04 LTS system
# @note Linux mainline incorporates an old `netx_uio` module by default. This script inspectes whether currently installed driver
#    provides expected memory maps and replaces it with a newly compiled one if not. If this does nto work for some reason (i.e.
#    driver installed in the system after running this script is still the old one) please remove kernel module under 
#    `/lib/modules/$(uname -r)/kernel/drivers/uio/uio_netx.ko` and run `depmod -a` when setting up the project.
#
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# Source BashUitils library
source $BASH_UTILS_HOME/source_me.bash

# ========================================================== Configruation ========================================================= #

# Script's log_info context
LOG_CONTEXT="cifx"

# ============================================================== Main ============================================================== #

main() {
    
    local ret

    # If UIO index of the CIFX device not defined, return immediately
    if ! is_var_set_non_empty NETX_UIO_INDEX; then
        exit 0
    fi
    
    # Check whether CIFX UIO (Userspace I/O) kernel module is present in the system
    if ls /lib/modules/$(uname -r)/kernel/drivers/uio/uio_netx.ko &> /dev/null; then
        
        # ----------------------------------------------------------------------------
        # If the module is present, check whether it is the new version of the driver
        # ----------------------------------------------------------------------------
    
        # Reload the driver
        sudo rmmod uio_netx    &> /dev/null || true
        sudo modprobe uio_netx &> /dev/null || true
    
        # Path to the UIO directory of the target device
        local NETX_UIO_DIR="/sys/class/uio/uio${NETX_UIO_INDEX}"

        # Check whether CIFX device is registered in the UIO system under the expected
        if ! ls $NETX_UIO_DIR &> /dev/null; then
            
            # If not, treat machine as not used to driver the robot and skip installation of the driver
            exit 0

        fi

        # Check whether CIFX device has expected UIO index
        if ! ls $NETX_UIO_DIR/name &> /dev/null || [[ $(cat $NETX_UIO_DIR/name) != "netx" ]]; then
            
            log_warn "CIFX device is expected to be loaded with the UIO index ${NETX_UIO_INDEX}, but either device is " \
                     "not presented or invalid 0'th device ($NETX_UIO_DIR) is loaded. CIFX UIO device driver module " \
                     "will not be installed/loaded."

            # Unload the driver
            sudo rmmod uio_netx &> /dev/null
            # Exit error
            exit 1

        fi
        
        # Check whether memory-mappings of the CIFX/netX card are present
        if ls $NETX_UIO_DIR/maps/map0/name &> /dev/null && ls $NETX_UIO_DIR/maps/map1/name &> /dev/null; then
            # Check whether memory-mappings of the CIFX/netX card are named as expected; if so, consider driver installed
            if [[ $(cat $NETX_UIO_DIR/maps/map0/name) == "dpm" && $(cat $NETX_UIO_DIR/maps/map1/name) == "dma" ]]; then
                
                # Unload the driver
                sudo rmmod uio_netx &> /dev/null
                # Exit error
                exit 0
            fi
        fi

    fi

    log_info "Installing cifx/netX Linux UIO driver..."

    # Create and enter temporary directory
    rm -rf /tmp/netx_uio
    mkdir /tmp/netx_uio
    cd /tmp/netx_uio

    log_info "Copying sources..."
    
    # Copy kernel sources
    cp $PROJECT_HOME/scripts/install/cifx/Makefile   ./Makefile
    cp $PROJECT_HOME/scripts/install/cifx/uio_netx.c ./uio_netx.c

    log_info "Patching sources..."

    # Fix the building problem
    sed -i 's/ioremap_nocache/ioremap_cache/g' uio_netx.c

    # Build the module
    log_info "Building kernel module"
    make

    # Install the module
    log_info "Installing module"
    sudo make modules_install

    # Update dependancies
    log_info "Updating dependencies"
    sudo depmod

    # Remove temporary directory
    cd ..
    rm -rf netx_uio

}

# ============================================================= Script ============================================================= #

# Run the script
source $BASH_UTILS_HOME/lib/scripting/templates/base.bash
