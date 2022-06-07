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

# Load the module into system
if ! lsmod | grep uio_netx > /dev/null; then
    log_info "Loading uio_netx module..."
    sudo modprobe uio_netx
    log_info "Module loaded"
fi
