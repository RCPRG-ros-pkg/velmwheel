/* ============================================================================================================================ *//**
 * @file       io.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 20th April 2022 8:06:44 pm
 * @modified   Wednesday, 25th May 2022 9:33:54 pm
 * @project    engineering-thesis
 * @brief      Definitions of public IO methods of the the EtherCAT Master class template
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __CIFX_ETHERCAT_MASTER_IMPL_IO_H__
#define __CIFX_ETHERCAT_MASTER_IMPL_IO_H__

/* =========================================================== Includes =========================================================== */

// System includes
#include <cstdint>
#include <mutex>
// Boost includes
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
// CIFX includes
#include "EcmIF_Public.h"
// Private includes
#include "cifx/ethercat/master.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace cifx {
namespace ethercat {

/* =========================================================== Constants ========================================================== */

/**
 * @brief Index of the data area of the CIFX device's channel used for EtherCAT communication
 * @details The @ref Matser driver utilizes onyl the 0'th area of the CIFX channel for EtherCAT
 *    communication. This is a 'regular' channel ( as opposed to 'priority' channel with idnex 
 *    @c 1 ). This is a default area used for communciation by CIFX EtherCAT firmware.
 * 
 * @see 'cifx_toolkit/doc/netx Dual-Port Memory Interface DPM 17 EN.pdf' (p.118/154)
 */
constexpr uint32_t CIFX_DATA_AREA_IDX = 0;

// Offset of the EtherCAT data within data image 
constexpr uint32_t CIFX_DATA_OFFSET = 0;

/* ================================================ Public methods (IO operations) ================================================ */

template<typename Lock>
void Master<Lock>::read_io() {

    std::lock_guard<Lock> glock(lock);
    
    /* --------------------------------- Read data ----------------------------------- */ 

    // Rad data from the channel
    auto status = xChannelIORead(
        cifx_channel,
        CIFX_DATA_AREA_IDX,
        CIFX_DATA_OFFSET,
        input_process_image.size(),
        input_process_image.data(),
        static_cast<uint32_t>(timeouts[to_underlying(TimeoutAction::ReadIO)].count())
    );

    // On error throw exception
    if(status != CIFX_NO_ERROR)
        throw cifx::Error{ status, "[cifx::ethercat::Master::read_io] Failed to read bus data" };

    /* ------------------------------- Dispatch data --------------------------------- */ 

    // Iterate over registered slaves to dispatch incoming data
    for(const auto &slave_record : slaves) {
        
        // Get pointer to the slave
        auto slave = slave_record.first;
        // Get reference to set of input PDOs registered by the slave
        const auto &pdo_references = slave_record.second.in;
        // Iterate over registered PDOs and update them
        for(std::size_t i = 0; i < pdo_references.size(); ++i)
            slave->update_input(i, pdo::get_reference(pdo_references[i]));
            
    }

}


template<typename Lock>
void Master<Lock>::write_io() {

    std::lock_guard<Lock> glock(lock);

    /* -------------------------------- Update data ---------------------------------- */ 

    // Iterate over registered slaves to update data to be written to the bus
    for(auto &slave_record : slaves) {
        
        // Get pointer to the slave
        auto slave = slave_record.first;
        // Get reference to set of output PDOs registered by the slave
        auto &pdo_references = slave_record.second.out;
        // Iterate over registered PDOs and update them
        for(std::size_t i = 0; i < pdo_references.size(); ++i)
            pdo::set_reference(pdo_references[i], slave->update_output(i));
            
    }
    
    /* --------------------------------- Read data ----------------------------------- */ 

    // Rad data from the channel
    auto status = xChannelIOWrite(
        cifx_channel,
        CIFX_DATA_AREA_IDX,
        CIFX_DATA_OFFSET,
        output_process_image.size(),
        output_process_image.data(),
        static_cast<uint32_t>(timeouts[to_underlying(TimeoutAction::WriteIO)].count())
    );

    // On error throw exception
    if(status != CIFX_NO_ERROR)
        throw cifx::Error{ status, "[cifx::ethercat::Master::write_io] Failed to read bus data" };

}


template<typename Lock>
std::vector<uint8_t> Master<Lock>::get_input_process_image() const noexcept {
    std::lock_guard<Lock> glock(lock);
    return input_process_image;
}


template<typename Lock>
std::vector<uint8_t> Master<Lock>::get_output_process_image() const noexcept {
    std::lock_guard<Lock> glock(lock);
    return output_process_image;
}

/* ================================================================================================================================ */

} // End namespace ethercat
} // End namespace cifx

#endif
