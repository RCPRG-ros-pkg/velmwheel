/* ============================================================================================================================ *//**
 * @file       ethercat_driver_impl.cpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 28th April 2022 12:31:55 pm
 * @modified   Monday, 13th June 2022 10:41:54 pm
 * @project    engineering-thesis
 * @brief      Definitions of the implementation class for the EtherCAT driver node of WUT Velmwheel robot
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

/* =========================================================== Includes =========================================================== */

// ROS includes
#include "pluginlib/class_loader.hpp"
// Private includes
#include "cifx/utilities.hpp"
#include "cifx/ethercat/utilities.hpp"
#include "velmwheel/utilities/enum.hpp"
#include "velmwheel/ethercat_driver_impl.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace velmwheel {

/* =========================================================== Constants ========================================================== */

// Index of the UIO device occupied by the CIFX card
constexpr int CIFX_UIO_NUM = 0;
// Index of the CIFX device's communication channel used by the EtherCAT driver
constexpr uint32_t CIFX_COMMUNICATION_CHANNEL_USED = 0;
// Name of the CIFX device registered to the CIFX Toolkit
constexpr auto CIFX_DEVICE_NAME = "cifx";

// Name of the bootloader file utilized by the driver
constexpr auto BOOTLOADER_NAME = "NETX100-BSL.bin";
// Name of the firmware file utilized by the driver
constexpr auto FIRMWARE_NAME = "cifxecm.nxf";

/* ===================================================== Public ctors & dtors ===================================================== */

EthercatDriverImpl::EthercatDriverImpl(
    rclcpp::Node &node,
    const Config &config
) :
    // Handle to the ROS interface
    node{ node },
    // CIFX Driver interfaces
    driver{ cifx::Driver::Config {
        .cos_polling_interval_ms   = config.driver_config.cos_polling_interval,
        .cos_polling_thread_params = config.driver_config.cos_polling_thread_config,
        .trace_level        = config.driver_config.log_level
    }},
    // CIFX device interface
    device { driver, CIFX_DEVICE_NAME, cifx::Device::Config {
        .uio_num = CIFX_UIO_NUM,
        .irq_thread_params = config.driver_config.irq_thread_config,
        .bootloader_file   = cifx::get_bootloader_path(BOOTLOADER_NAME),
        .firmware_file     = cifx::ethercat::get_firmware_path(FIRMWARE_NAME),
        .config_file       = config.eni_source
    }},
    // CIFX communication channel
    channel{ device, CIFX_COMMUNICATION_CHANNEL_USED },
    // EtherCAT master driver
    master{ channel },
    // Calculate bus cycle in [ms]
    bus_cycle_ms{ std::chrono::duration_cast<std::chrono::milliseconds>(master.get_bus_cycle()) }
{
    // Configue initial memory lock
    lock_memory(config.process_config.memory_allocation.lock_scheme);
    // Disable memory trimming, if requested
    if(config.process_config.memory_allocation.disable_trimming)
        disable_memory_trimming();
    if(config.process_config.memory_allocation.disable_mmap)
        disable_mmap();

    // Configure processing thread's paramaters
    cifx::set_thread_params(config.process_config.processing_thread);
}

/* ================================================ Public methods (configuration) ================================================ */


uint64_t EthercatDriverImpl::load_driver(std::string_view plugin) {

    // Create plugins-loader object
    pluginlib::ClassLoader<EthercatSlaveDriver> loader("velmwheel_ethercat_driver", "velmwheel::EthercatSlaveDriver");

    // Load the plugin
    std::shared_ptr<EthercatSlaveDriver> plugin_instance = loader.createSharedInstance(std::string{ plugin });

    // Initialize the plugin
    auto slave_names = plugin_instance->initialize(node);
    // Get slave interfaces for requested device
    std::vector<cifx::ethercat::Slave*> slaves;
    for(const auto &slave_name : slave_names)
        slaves.push_back(&master.get_slave(slave_name));
    // Configure the plugin
    plugin_instance->configure(slaves);

    // If configuration succeeded add instance to the set of loaded plugins
    this->slaves.emplace_back(
        SlaveRecord {
            .id    = next_unique_id,
            .name  = std::string{ plugin },
            .slave = plugin_instance
        }
    );
        
    /**
     * @note As 64-bit integer is used for drivers loading, the overload condition is just not
     *    probable and so it is not checked
     */

    // Return ID of the driver and increment it for the next one
    return next_unique_id++;
}


void EthercatDriverImpl::unload_driver(uint64_t id) {

    // Find slave with the given ID on the list of loaded slaves
    auto slave_record = std::find_if(slaves.begin(), slaves.end(), [id](const auto &slave){ return (slave.id == id); });
    // If slave not found, throw
    if(slave_record != slaves.end()) {
        std::stringstream ss;
        ss << "[EthercatDriverImpl::unload_driver] No driver with the given ID has been loaded "
           << "(" << id << ")";
        throw std::runtime_error { ss.str() };
    }

    // On success, unregister driver from the master
    slaves.erase(slave_record);

}


std::vector<EthercatDriverImpl::DriverDescriptor> EthercatDriverImpl::list_drivers() const {

    std::vector<DriverDescriptor> ret;

    // Reserve memory for the return vector
    ret.reserve(slaves.size());

    // Collect descriptions
    for(const auto &slave_record : slaves) {
        ret.push_back(DriverDescriptor{
            .id          = slave_record.id,
            .plugin_name = slave_record.name
        });
    }

    return ret;
}

/* ================================================================================================================================ */

} // End namespace velmwheel
