# ====================================================================================================================================
# @file       cifx.bash
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Wednesday, 22nd June 2022 12:08:51 pm
# @modified   Monday, 18th July 2022 5:52:44 pm
# @project    engineering-thesis
# @brief      Loads uio_netx kernel that maps Hilscher card's memory and interrupts into Linux UIO (Userspace I/O) subsystem. It is 
#             required by all drivers operating on these cards.
# 
# @copyright Krzysztof Pierczyk © 2022
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

        log_info "Adding '$USER' to '$CIFIX_GROUP_NAME' group..."
        
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

        local ret

        log_info "Adding udev rule for xifx/netX devices..."

        # Prepare udev rule providing access to the /dev/... file to members of the group and adding write access to the device's config file
        local RULE_DEV_FILTER="KERNEL==\"uio0\", SUBSYSTEM==\"uio\", ATTRS{maps/map0/name}==\"dpm\", ATTRS{maps/map1/name}==\"dma\", ACTION==\"add\", \\"
        local RULE_DEV_ACTION="    GROUP=\"$CIFIX_GROUP_NAME\", MODE=\"0664\", \\"
        # Prepare udev rule adding root access to the device's files in the /sys/class/uio/uioX directory
        local RULE_CFG_MOD_ACTION="    RUN+=\"/bin/chgrp -R $CIFIX_GROUP_NAME /sys%p\", RUN+=\"/bin/chmod -R g=u /sys%p\" \\"
        # Prepare udev rule adding root access to the device's configuration files in the /sys/class/uio/uioX/device directory
        local RULE_CFG_DEV_ACTION="    RUN+=\"/bin/chgrp -R $CIFIX_GROUP_NAME /sys%p/device/\", RUN+=\"/bin/chmod -R g=u /sys%p/device/\""

        # Create the file
        sudo touch $RULE_FILE_PATH &&
        # Add actual rule to the file
        echo "$RULE_DEV_FILTER"     | sudo tee    "$RULE_FILE_PATH" > /dev/null && 
        echo "$RULE_DEV_ACTION"     | sudo tee -a "$RULE_FILE_PATH" > /dev/null && 
        echo "$RULE_CFG_MOD_ACTION" | sudo tee -a "$RULE_FILE_PATH" > /dev/null && 
        echo "$RULE_CFG_DEV_ACTION" | sudo tee -a "$RULE_FILE_PATH" > /dev/null && 
        # Parse command statu
        ret=$? || ret=$?

        # If failed, report error
        if [[ $ret != 0 ]]; then
            log_error "Failed to create udev rule for cifx/netX devices"
        fi

        log_info "Udev rule has been sucesfully added"

        # Reload udev rules
        sudo udevadm control --reload-rules > /dev/null && sudo udevadm trigger > /dev/null || {
            log_error "Failed to reload udev rules"
            return
        }

        log_info "Udev rules has been reloaded. Technically, you shouldn't need to reboot the system..."
        log_info "(But probably you should)"

    fi

}

verify_cifx_driver() {

    # Path to the UIO directory of the target device
    local NETX_UIO_DIR="/sys/class/uio/uio${NETX_UIO_INDEX}"

    # Check whether memory-mappings of the CIFX/netX card are named as expected; if so, consider driver invalid
    if ! ls $NETX_UIO_DIR/maps/map0/name &> /dev/null || [[ $(cat $NETX_UIO_DIR/maps/map0/name) != "dpm" ]] || 
        ! ls $NETX_UIO_DIR/maps/map1/name &> /dev/null || [[ $(cat $NETX_UIO_DIR/maps/map1/name) != "dma" ]];
    then
        return 1
    # Otherwise, return success
    else
        return 0
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

        # Load the driver
        sudo modprobe uio_netx
        
        # Check whether memory-mappings of the CIFX/netX card are named as expected; if so, consider driver invalid
        if ! verify_cifx_driver; then
        
            log_info "Unloading legacy driver..."

            # Unload the driver
            sudo rmmod uio_netx
            # Exit error
            return 1
        fi
        
        log_info "Driver module loaded"

    # If driver already loaded, verify it
    else
        
        # Check whether memory-mappings of the CIFX/netX card are named as expected; if so, consider driver invalid
        if ! verify_cifx_driver; then
        
            log_info "Unloading legacy driver..."

            # Unload the driver
            sudo rmmod uio_netx
            # Exit error
            return 1
        fi
        
    fi

    return 0
}

# ============================================================== Main ============================================================== #

main() {

    # If UIO index of the CIFX device not defined, return immediately
    if ! is_var_set_non_empty NETX_UIO_INDEX; then
        exit 0
    fi

    # Add dedicated group for accessing cifx/netX devices in the sysfs and add current user to it
    add_cifx_group

    # Add udev rule adding cifx/netX device to the created group every time the device is added to the system
    add_cifx_udev_rule

    # Load cifx UIO driver to the kernel (if routine faield due to legacy driver being loaded, rerun it to reload the driver)
    if ! load_cifx_driver; then
        load_cifx_driver
    fi
}

# ============================================================= Script ============================================================= #

# Run the script
source $BASH_UTILS_HOME/lib/scripting/templates/base.bash
