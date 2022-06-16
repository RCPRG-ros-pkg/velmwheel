# ====================================================================================================================================
# @file       rt_kernel.bash
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Wednesday, 15th June 2022 11:22:19 pm
# @modified   Thursday, 16th June 2022 12:57:31 am
# @project    Winder
# @brief      Script automating building and installation of the RT-patched Linux kernel
#    
#    
# @see https://docs.ros.org/en/foxy/Tutorials/Building-Realtime-rt_preempt-kernel-for-ROS-2.html
# @copyright Krzysztof Pierczyk © 2021
# ====================================================================================================================================

# Source bash-utils library
source $BASH_UTILS_HOME/source_me.bash

# ========================================================== Configruation ========================================================= #

# Script's log_info context
LOG_CONTEXT="rt-kernel"

# ============================================================== Main ============================================================== #

main() {
    
    # Target version of the kernel
    local KERNEL_VERSION='5.15.44'
    # Target subversion of the RT patch
    local RT_PATCH_SUBVERSION='46'

    # URL to the kernel
    local KERNEL_URL="https://mirrors.edge.kernel.org/pub/linux/kernel/v${KERNEL_VERSION%%.*}.x/linux-${KERNEL_VERSION}.tar.gz"
    # URL to the patch
    local RT_PATCH_URL="https://cdn.kernel.org/pub/linux/kernel/projects/rt/${KERNEL_VERSION%.*}/patch-${KERNEL_VERSION}-rt${RT_PATCH_SUBVERSION}.patch.gz"

    # -------------------------------- Download sources -------------------------------- 

    log_info "Building $KERNEL_VERSION kernel with a ${KERNEL_VERSION}-rt${RT_PATCH_SUBVERSION} patch..."

    pushd /tmp > /dev/null

    # Create build directory 
    rm -rf kernel
    mkdir -p kernel && cd ./kernel || {
        log_error "Failed to create build directory"
        return 1
    }
    
    log_info "Downloading kernel sources..."

    # Download kernel
    wget "$KERNEL_URL" || {
        log_error "Failed to download kernel sources"
        return 1
    }

    log_info "Downloading RT patch sources..."

    # Download patch
    wget "$RT_PATCH_URL" || {
        log_error "Failed to download RT patch sources"
        return 1
    }

    # Name of the downloaded kernel
    local KERNEL_NAME=${KERNEL_URL##*/}
    # Name of the downloaded patch
    local RT_PATCH_NAME=${RT_PATCH_URL##*/}
    
    log_info "Unpacking kernel sources..."

    # Unpack kernel
    tar -xzf "$KERNEL_NAME" || {
        log_error "Failed to unpack kernel sources"
        return 1
    }
    
    log_info "Unpacking RT patch sources..."
    
    # Unpack patch
    gunzip "$RT_PATCH_NAME" || {
        log_error "Failed to unpck RT patch sources"
        return 1
    }

    # Name of the unpacked kernel
    local KERNEL_UNPACKED_NAME=${KERNEL_NAME%.tar.gz}
    # Name of the unpacked patch
    local RT_PATCH_UNPACKED_NAME=${RT_PATCH_NAME%.gz}

    # ---------------------------------- Patch kernel ----------------------------------

    # Enter kernel directory
    cd "$KERNEL_UNPACKED_NAME"
    
    log_info "Patching kernel sources..."

    # Patch source files
    patch -p1 < "../$RT_PATCH_UNPACKED_NAME" || {
        log_error "Failed to patch kernel sources"
        return 1
    }

    # --------------------------------- Prepare config --------------------------------- 

    # Copy default config
    cp /boot/config-$(uname -r) .config &> /dev/null || {
        log_error "Failed to copy default system config from '/boo/config-$(uname -r)'"
        return 1
    }
    
    log_info "Updating default configuration..."

    # Enable Ubuntu configurations
    yes '' | make oldconfig && ret=$? || ret=$?
    # Enable required configuration
    make menuconfig

    # ---------------------------------- Build kernel ---------------------------------- 

    log_info "Building kernel..."

    # Build kernel
    make -j $(nproc) deb-pkg || {
        log_error "Failed to build the kernel"
        return 1
    }

    log_info "Kernel built sucesfully"
    log_info "Installing new kernel..."

    # Install kernel
    sudo dpkg -i ../*.deb || {
        log_error "Failed to install the kernel"
        return 1
    }

    log_info "Kernel v.$KERNEL_VERSION with a ${KERNEL_VERSION}-rt${RT_PATCH_SUBVERSION} patch has been sucesfully installed"
}

# ============================================================= Script ============================================================= #

# Run the script
source $BASH_UTILS_HOME/lib/scripting/templates/base.bash
