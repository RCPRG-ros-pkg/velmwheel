/* ============================================================================================================================ *//**
 * @file       ethercat_driver_impl.cpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 28th April 2022 12:31:55 pm
 * @modified   Friday, 27th May 2022 6:05:34 pm
 * @project    engineering-thesis
 * @brief      Definitions of the implementation class for the EtherCAT driver node of WUT Velmwheel robot
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __VELMWHEEL_ETHERCAT_DRIVER_IMPL_ETHERCAT_DRIVER_IMPL_H__
#define __VELMWHEEL_ETHERCAT_DRIVER_IMPL_ETHERCAT_DRIVER_IMPL_H__

/* =========================================================== Includes =========================================================== */

// System includes
#include <sys/mman.h>
#include <malloc.h>
// Private includes
#include "velmwheel/ethercat_driver_impl.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace velmwheel {

/* ============================================ Public methods (process configuration) ============================================ */

bool EthercatDriverImpl::lock_memory(MemoryLockScheme scheme) {
    switch(scheme) {
        case MemoryLockScheme::Current: return (not mlockall( MCL_CURRENT              ));
        case MemoryLockScheme::All:     return (not mlockall( MCL_CURRENT | MCL_FUTURE ));
        default:
            return true;
    }    
}

bool EthercatDriverImpl::disable_memory_trimming() {
    return (mallopt(M_TRIM_THRESHOLD, -1) != 0);
}


bool EthercatDriverImpl::disable_mmap() {
    return (mallopt(M_MMAP_MAX, 0) != 0);
}

/* ================================================= Public methods (bus control) ================================================= */

void EthercatDriverImpl::set_bus_enabled(bool enabled) {
    using namespace std::literals::chrono_literals;
    channel.set_bus_on(enabled, 100ms);
}


bool EthercatDriverImpl::is_bus_enabled() {
    using namespace std::literals::chrono_literals;
    return channel.is_bus_on(100ms);
}


void EthercatDriverImpl::set_master_target_state(ethercat::MasterState target_state) {
    using namespace std::literals::chrono_literals;
    master.set_state(target_state, 100ms);
}


ethercat::MasterStateInfo EthercatDriverImpl::get_master_state() {
    using namespace std::literals::chrono_literals;
    return master.get_state_info(100ms);
}


ethercat::TimingInfo EthercatDriverImpl::get_timing_info() {
    using namespace std::literals::chrono_literals;
    return master.get_timing_info(5s);
}

/* ====================================================== Public methods (io) ===================================================== */

template<typename HandlerT>
void EthercatDriverImpl::set_bus_event_handler(Event event, HandlerT &&handler) {
    switch(event) {
        case Event::ReadBusReady: 
            channel.get_process_data(
                cifx::ethercat::Master::CIFX_PDI_DATA_AREA
            ).register_notification<cifx::ProcessData::Event::Input>(handler); break;
        case Event::WriteBusRead:
            channel.get_process_data(
                cifx::ethercat::Master::CIFX_PDI_DATA_AREA
            ).register_notification<cifx::ProcessData::Event::Output>(handler); break;
        default:
            return;
    }
}


void EthercatDriverImpl::read_bus() {
    master.read_bus(bus_cycle_ms);
}


void EthercatDriverImpl::write_bus() {
    master.write_bus(bus_cycle_ms);
}

/* ================================================================================================================================ */

} // End namespace velmwheel

#endif
