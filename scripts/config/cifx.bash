# ====================================================================================================================================
# @ Filename: cifx.bash
# @ Author: Krzysztof Pierczyk
# @ Create Time: 2021-01-14 21:29:40
# @ Modified time: 2021-01-14 21:52:44
# @ Description: 
#    
#    Loads uio_netx kernel that maps Hilscher card's memory and interrupts into Linux UIO (Userspace I/O) subsystem. It is 
#    required by all drivers operating on these cards.
#    
# ====================================================================================================================================

# Source BashUitils library
source $BASH_UTILS_HOME/source_me.bash

# ============================================================= Script ============================================================= #

# Script's log context
LOG_CONTEXT="cifx"

# Name of the group that will allow access to cifx/netX device in the sysfs (/sys)
CIFIX_GROUP_NAME="cifxers"

# ============================================================ Functions =========================================================== #

add_cifx_group() {

    # Check if 'cifxers' group that will allow access to cifx/netX device in the sysfs (/sys)
    if ! getent group "$CIFIX_GROUP_NAME" > /dev/null; then

        log_info "Creating '$CIFIX_GROUP_NAME' for cifx/netX devices access..."

        # Create the group
        if ! sudo groupadd "$CIFIX_GROUP_NAME" &> /dev/null; then
            log_error "Failed to create the group"
            return
        fi

        log_info "Group has been sucesfully created"

    fi

    # Check if user is in the group already
    if ! groups "$USER" | grep -q "\\b$CIFIX_GROUP_NAME\\b"; then

        log_info "Adding '$USER' to '$CIFIX_GROUP_NAME'group..."
        
        # Add current user to the group 
        if ! sudo usermod -a -G "$CIFIX_GROUP_NAME" "$USER"; then
            log_error "Failed to add user to the group"
            return
        fi

        log_info "User has been added to the group"
        
    fi
}

add_cifx_udev_rule() {

    local RULE_FILE_PATH="/etc/udev/rules.d/99-cifx.rules"

    # Check if rule file has been already added
    if ! ls "$RULE_FILE_PATH" &> /dev/null; then

        LOG_CONTEXT="cifx-todo" \
            log_warn "Add actual udev rule creation to the scripts/config/cifx.bash script!"

    fi

}

load_cifx_driver() {

    # Check if the netX kernel module is available
    if ! ls /lib/modules/$(uname -r)/kernel/drivers/uio/uio_netx.ko &> /dev/null; then
        log_warn "No cifx/netX  Linux kernel module found in the system"
        exit 0
    fi

    # Load the module into system
    if ! lsmod | grep uio_netx > /dev/null; then
        log_info "Loading uio_netx module..."
        sudo modprobe uio_netx
        log_info "Module loaded"
    fi
}

# ============================================================== Main ============================================================== #

main() {

    # Add dedicated group for accessing cifx/netX devices in the sysfs and add current user to it
    add_cifx_group

    # Add udev rule adding cifx/netX device to the created group every time the device is added to the system
    add_cifx_udev_rule

    # Load cifx UIO driver to the kernel
    load_cifx_driver
}

# ============================================================= Script ============================================================= #

# Run the script
source $BASH_UTILS_HOME/lib/scripting/templates/base.bash
