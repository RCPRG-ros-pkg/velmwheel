/* ============================================================================================================================ *//**
 * @file       ethercat_driver_impl.cpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 28th April 2022 12:31:55 pm
 * @modified   Thursday, 30th June 2022 2:17:30 pm
 * @project    engineering-thesis
 * @brief      Definitions of the implementation class for the EtherCAT driver node of WUT Velmwheel robot
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

/* =========================================================== Includes =========================================================== */

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
    // Host state guard
    state_guard{ channel },

    // EtherCAT master driver
    master{ channel },
    // Calculate cyclic I/O timeout [ms] (use value slightly higher than the bus cycle to given CIFX toolkit time buffer for processing)
    cyclic_io_timeout_ms{ 
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::microseconds{
                static_cast<int64_t>(master.get_bus_cycle().count() * 1.2)
            }
        ) 
    },

    /**
     * @todo Does setting @a cyclic_io_timeout_ms to 1.2 [cycle-time] makes sense? Should it be rather
     *    0.5 [cycle-time]? The latter approach would skip input data that are not fed from the bus into 
     *    the buffer in time.
     */

    //  Slaves UP timeout
    slaves_up_timeout{ config.slaves_up_timeout.has_value() ?

        // If timeout given explicitly, set it
        *config.slaves_up_timeout :

        /**
         * Otherwise, set default timeout
         * 
         * @note This default value is choosen based on results of some practical tests
         */
        std::chrono::duration_cast<std::chrono::milliseconds>(master.get_bus_cycle()) * 2'500
        
    },

    // Create class loader for drivers plugins
    loader{ "velmwheel_ethercat_driver", "velmwheel::EthercatSlaveDriver" }
{
    /* --------------------------------- Configure bus ------------------------------- */

    // Configure CIFX synchronisation mode to IO2
    master.set_sync_mode(cifx::ethercat::Master::SyncMode::IO2, std::chrono::seconds{ 5 });

    // Get list of all slaves names
    auto slaves_names = master.list_slaves();
    // Get list of all slaves
    auto slaves = master.get_slaves();
    
    // Calculate end of period available to slaves for booting up into the Operational state
    auto state_wait_end = node.get_clock()->now() + slaves_up_timeout;

    // Wait for all slaves to be brought into the Operational state
    while(true){

        // Time between subsequent checks of slaves' states
        constexpr std::chrono::seconds STATE_CHECK_PERIOD{ 2 };

        std::vector<cifx::ethercat::Slave::State> states(slaves.size());

        // Read state of all slaves
        std::transform(slaves.begin(), slaves.end(), states.begin(),
            [](auto const &slave) { return slave->get_state(); }
        );

        // Check whether all slaves has been brough to the Operation state
        bool all_slaves_up = std::all_of(states.begin(), states.end(),
            [](auto const &st) { return (st == cifx::ethercat::Slave::State::Op); }
        );

        // If all slaves up, break loop
        if(all_slaves_up)
            break;

        // If timeout occurred, throw error
        if(node.get_clock()->now() > state_wait_end) {
            RCLCPP_ERROR_STREAM(node.get_logger(), "Not all slaves has been brought into the Operational slaves before configured timeout");
            throw std::runtime_error{ "Not all slaves has been brought into the Operational slaves before configured timeout" };
        }

        // Otherwise, print log and wait for the next check iteration
        RCLCPP_INFO_STREAM(node.get_logger(), "Waiting " << STATE_CHECK_PERIOD.count() << " [sec] for slave devices enter Operational state. Current states:");
        for(std::size_t i = 0; i < states.size(); ++i)
            RCLCPP_INFO_STREAM(node.get_logger(), " - " << slaves_names[i] << ": " << cifx::ethercat::Slave::state_to_str(states[i]));
        // Wait for the next iteration
        std::this_thread::sleep_for(STATE_CHECK_PERIOD);

    };

    /* ------------------------ Configure process parameters ------------------------- */

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
