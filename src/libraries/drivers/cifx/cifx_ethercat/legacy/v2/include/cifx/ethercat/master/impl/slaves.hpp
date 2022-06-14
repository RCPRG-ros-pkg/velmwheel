/* ============================================================================================================================ *//**
 * @file       slaves.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 20th April 2022 8:06:44 pm
 * @modified   Thursday, 28th April 2022 11:23:35 am
 * @project    engineering-thesis
 * @brief      Definitions of public slaves-related methods of the the EtherCAT Master class template
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __CIFX_ETHERCAT_MASTER_IMPL_SLAVES_H__
#define __CIFX_ETHERCAT_MASTER_IMPL_SLAVES_H__

/* =========================================================== Includes =========================================================== */

// System includes
#include <cstdint>
#include <mutex>
// Boost includes
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
// Private includes
#include "cifx/ethercat/master.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace cifx {
namespace ethercat {

/* =============================================== Protected methods (slave drivers) ============================================== */

template<typename Lock>
void Master<Lock>::register_slave(std::shared_ptr<Slave> slave) {

    std::lock_guard<Lock> glock(lock);
    
    // Check if valid pointer given
    if(slave == nullptr)
        throw cifx::Error{ CIFX_INVALID_PARAMETER, "[cifx::ethercat::register_slave] An empty poitner given" };
    
    // Get description of PDOs that the slave advertises
    auto pdos_description = slave->get_pdos_description(eni_path);
    // Create lists of PDO references for the slave
    pdo::ReferencesSet pdos_reference;
    // Reserve memory for refernces sets
    pdos_reference.in.reserve(pdos_description.in.size());
    pdos_reference.out.reserve(pdos_description.out.size());

    // Parse input PDOs
    for(const auto in_description : pdos_description.in) {

        // Calculate size of the PDO
        std::size_t pdo_size = pdo::data_type_to_size(in_description.type);
        // Check if PDO is mapped to a valid Process Data Image region
        if(in_description.offset + pdo_size > input_process_image.size())
            throw cifx::Error{ CIFX_INVALID_PARAMETER, "[cifx::ethercat::register_slave] Invalid mapping region of the input PDO" };
        // Construct PDO reference for the slave
        pdos_reference.in.push_back(pdo::Reference{
            .data{ &input_process_image[in_description.offset], static_cast<ranges::span<uint8_t>::index_type>(pdo_size) },
            .type{ in_description.type                                                                                   }
        });
        
    };

    // Parse output PDOs
    for(const auto out_description : pdos_description.out) {

        // Calculate size of the PDO
        std::size_t pdo_size = pdo::data_type_to_size(out_description.type);
        // Check if PDO is mapped to a valid Process Data Image region
        if(out_description.offset + pdo_size > output_process_image.size())
            throw cifx::Error{ CIFX_INVALID_PARAMETER, "[cifx::ethercat::register_slave] Invalid mapping region of the output PDO" };
        // Construct PDO reference for the slave
        pdos_reference.out.push_back(pdo::Reference{
            .data{ &output_process_image[out_description.offset], static_cast<ranges::span<uint8_t>::index_type>(pdo_size) },
            .type{ out_description.type                                                                                    }
        });
        
    };

    // If PDO parsing succeeded, ad slave to the registration list
    slaves[slave] = pdos_reference;
    // Call slave-specific registration callback
    slave->on_registration();
}


template<typename Lock>
void Master<Lock>::unregister_slave(std::shared_ptr<Slave> slave) {
    std::lock_guard<Lock> glock(lock);
    unregister_slave_impl(slave);
}

/* ============================================== Protected methods (implementations) ============================================= */

template<typename Lock>
void Master<Lock>::unregister_slave_impl(std::shared_ptr<Slave> slave) {

    // Check if valid pointer given
    if(slave == nullptr)
        throw cifx::Error{ CIFX_INVALID_PARAMETER, "[cifx::ethercat::unregister_slave_impl] An empty poitner given" };
    // Check if registered slave given
    if(slaves.find(slave) == slaves.end())
        throw cifx::Error{ CIFX_INVALID_PARAMETER, "[cifx::ethercat::unregister_slave_impl] Cannot unregister non-registered slave" };

    // Call slave-specific on-unregistration routine
    slave->on_unregistration();

    // Get reference to set of output PDOs registered by the slave
    auto &out_pdo_references = slaves[slave].out;
    // Iterate over registered output PDOs and update them
    for(std::size_t i = 0; i < out_pdo_references.size(); ++i)
        pdo::set_reference(out_pdo_references[i], slave->update_output(i));

}

/* ================================================================================================================================ */

} // End namespace ethercat
} // End namespace cifx

#endif

