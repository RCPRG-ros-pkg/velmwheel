/* ============================================================================================================================ *//**
 * @file       timing.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Tuesday, 26th April 2022 12:08:27 pm
 * @modified   Tuesday, 26th April 2022 1:21:08 pm
 * @project    engineering-thesis
 * @brief      Declaration of data structures related to bus timing
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __CIFX_MASTER_TIMING_H__
#define __CIFX_MASTER_TIMING_H__

/* =========================================================== Includes =========================================================== */

#include <chrono>

/* ========================================================== Namespaces ========================================================== */

namespace cifx {
namespace ethercat {

/* ============================================================= Types ============================================================ */

/**
 * @brief Synchronisation modes of the EtherCAT bus 
 * @see 'doc/EtherCAT Master V4 Protocol API 06 EN.pdf'
 */
enum class SyncMode {
    FreeRun,
    IO1,
    IO2
};

/**
 * @brief Enumeration of timeouts configurable in the driver
 */
enum class TimeoutAction : std::size_t {
    DeviceCommunication = 0,
    PutPacket           = 1,
    GetPacket           = 2,
    ReadIO              = 3,
    WriteIO             = 4,
    Num                 = 5
};

/**
 * @brief Description of timing parameters of the bus
 */
struct TimingInfo {

    // Duration of the bus cycle
    std::chrono::nanoseconds bus_cycle;
    // Duration of the frame transmition
    std::chrono::nanoseconds frame_transmition_time;
    // Expected bus delay
    std::chrono::nanoseconds expected_bus_delay;
    // Expected time of the RX transaction end (from start of bus cycle transmission)
    std::chrono::nanoseconds expected_rx_end_time; 
    // Expected time of the TX transaction end (from start of bus cycle transmission)
    std::chrono::nanoseconds expected_tx_end_time; 
    
};

/* ================================================================================================================================ */

} // End namespace ethercat
} // End namespace cifx

#endif
