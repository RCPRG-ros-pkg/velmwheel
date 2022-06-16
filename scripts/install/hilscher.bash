# ====================================================================================================================================
# @file       hilscher.bash
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Thursday, 14th January 2021 9:29:40 pm
# @modified   Thursday, 16th June 2022 12:22:49 pm
# @project    engineering-thesis
# @brief      Downloads, builds, installs and loads uio_netx kernel that maps Hilscher card's memory and interrupts into Linux
#             UIO (Userspace I/O) subsystem. It is required by all drivers operating on these cards.
#
# @note UIO NETX V2.0.1.0 has problem building as ioremap_nocache() is no longer present in the Linux'es sources. This
#    script changes all calls to this function into calls to ioremap_cache().
# @note Before running this script `sourceMe.bash` (under project's home directory) should be sourced by the calling terminal
# @note Script was tested on Ubuntu 20.0.4 LTS system
# @note Linux mainline incorporates an old `netx_uio` module by default. Before running this script please remove kernel
#    module under `/lib/modules/$(uname -r)/kernel/drivers/uio/uio_netx.ko` and run `depmod -a` when setting up the project on
#    a newly installed Linux.
#
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# Source BashUitils library
source $BASH_UTILS_HOME/source_me.bash

# ========================================================== Configruation ========================================================= #

# Script's log_info context
LOG_CONTEXT="cifx"

# URL to the netx UIO module
declare NETX_UIO_URL='https://kb.hilscher.com/download/attachments/121271969/uio_netx-2.0.1.0.tar.bz2?api=v2'

# ============================================================== Main ============================================================== #

main() {
    
    local ret

    # Check whether CIFX UIO (Userspace I/O) kernel module is present in the system
    sudo ls /lib/modules/$(uname -r)/kernel/drivers/uio/uio_netx.ko &> /dev/null && ret=$? || ret=$?

    # If `ls` didn;t find the module, install it
    if [[ $ret == 2 ]] ; then

        log_info "Installing cifx/netX Linux UIO driver..."

        # Create and enter temporary directory
        rm -rf /tmp/netx_uio
        mkdir /tmp/netx_uio
        cd /tmp/netx_uio

        # Download kernel module
        log_info "Downloading the module"
        wget "$NETX_UIO_URL" -O netx_uio.tar.bz2 || {
            log_error "Failed to download cifx/netX Linux UIO driver"
            return 1
        }

        # Extract files
        log_info "Extracting files"
        tar -xf netx_uio.tar.bz2 > /dev/null
        rm netx_uio.tar.bz2
        mv ./*/* ./

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

    # If it failed for any other reason, print error
    elif [[ $ret != 0 ]]; then
        log_error "Failed to verify whether CIFX/netX UIO module is installed on the system"
    fi

}

# ============================================================= Script ============================================================= #

# Run the script
source $BASH_UTILS_HOME/lib/scripting/templates/base.bash
