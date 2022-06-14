/* ============================================================================================================================ *//**
 * @file       master.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 20th April 2022 8:06:44 pm
 * @modified   Wednesday, 27th April 2022 3:45:31 pm
 * @project    engineering-thesis
 * @brief      Definitions of public ctors & dtors of the the EtherCAT Master class template
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __CIFX_ETHERCAT_MASTER_IMPL_MASTER_H__
#define __CIFX_ETHERCAT_MASTER_IMPL_MASTER_H__

/* =========================================================== Includes =========================================================== */

// System includes
#include <cstdint>
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

/* ===================================================== Public ctors & dtors ===================================================== */

template<typename Lock>
Master<Lock>::Master(
    cifx::Toolkit &toolkit,
    std::string_view device_name,
    uint32_t channel_id
) :
    // Initialize CIFX interface
    cifx_driver{ toolkit.open_driver() },
    cifx_channel{ toolkit.open_channel(device_name, cifx_driver, channel_id) },
    // Initialize const parameters
    eni_path{ },
    bus_cycle{ }
{

    /* ---------------------- Get path to the ENI configuration ---------------------- */

    // Get pointer to the CIFX device registered in the toolkit
    PDEVICEINSTANCE cifx_device = xDeviceGet();
    // Check if device has been proeprly initialized
    if(!cifx_device)
        throw cifx::Error{ CIFX_INVALID_POINTER, "[cifx::ethercat::Master] CIFX device not initialize din the Toolkit" };

    CIFX_DEVICE_INFORMATION dev_info { };

    // Prepare device information structure for the toolkit
    dev_info.ulDeviceNumber   = cifx_device->ulDeviceNumber;
    dev_info.ulSerialNumber   = cifx_device->ulSerialNumber;
    dev_info.ulChannel        = channel_id;
    dev_info.ptDeviceInstance = cifx_device;

    CIFX_FILE_INFORMATION eni_file_info { };

    /**
     * @brief Index of the configuration file (ENI) associated with the CIFX device
     * @note Function always gets 0'th configuration file as it is
     *   the only one supported by the current CIFX driver implementation
     */
    constexpr uint32_t CONFIG_FILE_INDEX = 0;
    
    // Try to get configuration file
    auto ret = USER_GetConfigurationFile(&dev_info, CONFIG_FILE_INDEX, &eni_file_info);

    // Check if file has been succesfully obtained
    if(!ret)
        throw cifx::Error{ CIFX_FUNCTION_FAILED, "[cifx::ethercat::Master] Failed to obtain ENI configuration for the CIFX device" };

    // Cache path to the file
    const_cast<std::string&>(eni_path) = std::string{ eni_file_info.szFullFileName };

    /* ---------------------------- Read ENI configuration --------------------------- */

    boost::property_tree::ptree xml_tree;
    
    // Load XML ENI file
	boost::property_tree::read_xml(eni_path, xml_tree);

    /* -------------------- Parse timing configruation of the bus -------------------- */
    
    // Path of the ENI tag containing bus cycle in [us]
    constexpr auto ENI_BUS_CYCLE_TAG = "EtherCATConfig.Config.Cyclic.CycleTime";

    // Parse bus cycle tag
    auto bus_cycle_tag = xml_tree.get_child(ENI_BUS_CYCLE_TAG);
    // Parse bus cycle 
    const_cast<std::chrono::nanoseconds&>(bus_cycle) = std::chrono::nanoseconds{ bus_cycle_tag.get_value<unsigned>() };

    /* -------------------- Initialize process data image buffers -------------------- */

    // Path of the ENI tag containing size of the input Process Data Image (PDI) in bytes
    constexpr auto ENI_INPUT_PROCESS_IMAGE_SIZE_TAG = "EtherCATConfig.Config.ProcessImage.Inputs.ByteSize";
    // Path of the ENI tag containing size of the output Process Data Image (PDI) in bytes
    constexpr auto ENI_OUTPUT_PROCESS_IMAGE_SIZE_TAG = "EtherCATConfig.Config.ProcessImage.Outputs.ByteSize";

    // Parse PDI size tags
    auto input_pdi_tag  = xml_tree.get_child(ENI_INPUT_PROCESS_IMAGE_SIZE_TAG);
    auto output_pdi_tag = xml_tree.get_child(ENI_OUTPUT_PROCESS_IMAGE_SIZE_TAG);
    // Parse PDI size 
    auto input_pdi_size = input_pdi_tag.get_value<unsigned>();
    auto output_pdi_size = output_pdi_tag.get_value<unsigned>();
    // Initialize PDI buffers
    input_process_image.resize(input_pdi_size, 0);
    output_process_image.resize(output_pdi_size, 0);

}


template<typename Lock>
Master<Lock>::~Master() {

    // Unregister all slaves to close bus gently
    try {
        
        for(const auto &slave_entry : slaves)
            unregister_slave_impl(slave_entry.first);
            
    } catch(...) { }

    // Deactivate bus gently
    try {

        // Set bus unactive
        set_bus_on_impl(false);
        // Set master unready in the CIFX device
        set_master_ready_impl(false);
        
    } catch(...) { }
        
}

/* ================================================================================================================================ */

} // End namespace ethercat
} // End namespace cifx

#endif
