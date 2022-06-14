/* ============================================================================================================================ *//**
 * @file       ethercat_driver_node.cpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 28th April 2022 12:31:55 pm
 * @modified   Tuesday, 14th June 2022 4:25:19 pm
 * @project    engineering-thesis
 * @brief      Definitions of the EtherCAT driver node of WUT Velmwheel robot
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

/* =========================================================== Includes =========================================================== */

// Private includes
#include "node_common/node.hpp"
#include "velmwheel/utilities/enum.hpp"
#include "velmwheel/ethercat_driver.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace velmwheel {

/* =========================================================== Constants ========================================================== */

// List of acceptable values of the initial memory lock scheme parameter
constexpr std::array VALID_INITIAL_MEMORY_LOCKS { "none", "current", "all" };
// List of acceptable values of the CIFX log level
constexpr std::array VALID_CIFX_LOG_LEVELS { "Debug", "Info", "Warning", "Error" };
// List of acceptable values of scheduling policty
constexpr std::array VALID_SCHED_POLICIES { "SCHED_OTHER", "SCHED_FIFO", "SCHED_RR" };

/* ====================================================== Auxiliary functions ===================================================== */

/**
 * @brief Helper algorithm checking whether @p container contains @p element
 */
template<typename T, typename E>
inline bool contains(const T &container, const E &element) {
    return (std::find(container.begin(), container.end(), element) == container.end());
}

/**
 * @brief Parses @p description of the ENI file into the absolute path to the file
 * 
 * @param description 
 *    description of the ENI resource
 * 
 * @see EthercatDriverImpl::Config::eni_source
 * 
 * @throws std::runtime_error
 *    if invalid description has been given
 */
static inline std::filesystem::path parse_eni_config_path(const std::vector<std::string> &description) {

    // Check if description of valid size given
    if(description.size() != 1 && description.size() != 3)
        throw std::runtime_error{ "[velmwheel::parse_eni_config_path] Invalid format of the ENI resource description" };

    // If single string given, treat it as an absolute path and return
    if(description.size() == 1)
        return description[0];

    // Otherwise, parse path from resource idnex
    return package_common::resources::get_file_path(description[0], description[1], description[2]);
};

/**
 * @brief Helper method transforming human-readable name of the memory lock 
 *   scheme into driver-specific enumeration
 */
inline MemoryLockScheme parse_locking_scheme(std::string_view locking_scheme) {
         if(locking_scheme == "none")    return MemoryLockScheme::None;
    else if(locking_scheme == "current") return MemoryLockScheme::Current;
    else if(locking_scheme == "all")     return MemoryLockScheme::All;
    else
        throw std::runtime_error{ "[EthercatDriver] Invalid memory locking scheme" };
}

/**
 * @brief Helper method transforming human-readable name of the CIFX log
 *   level into driver-specific enumeration
 */
inline ethercat::LogLevel parse_log_level(std::string_view log_level) {
         if(log_level == "Debug")   return ethercat::LogLevel::Debug;
    else if(log_level == "Info")    return ethercat::LogLevel::Info;
    else if(log_level == "Warning") return ethercat::LogLevel::Warning;
    else if(log_level == "Error")   return ethercat::LogLevel::Error;
    else
        throw std::runtime_error{ "[EthercatDriver] Invalid CIFX log level" };
}

/**
 * @brief Helper method transforming IRQ polling interval from optional double
 *    expressing amount of seconds into optional chrono duration
 */
inline std::optional<std::chrono::milliseconds> parse_polling_interval(std::optional<double> interval_s) {
    if(interval_s.has_value())
        return std::optional<std::chrono::milliseconds>{  static_cast<std::chrono::milliseconds::rep>((*interval_s) * 1'000) };
    else
        return std::optional<std::chrono::milliseconds>{  };
}

/**
 * @brief Helper method transforming human-readable name of the scheduling
 *   policy into driver-specific enumeration
 */
inline ThreadSchedPolicy parse_sched_policy(std::string_view policy) {
         if(policy == "SCHED_OTHER") return ThreadSchedPolicy::Other;
    else if(policy == "SCHED_FIFO")  return ThreadSchedPolicy::Fifo;
    else if(policy == "SCHED_RR")    return ThreadSchedPolicy::RoundRobin;
    else
        throw std::runtime_error{ "[EthercatDriver] Invalid scheduling policy" };
}

/**
 * @brief Converts chrono duration into builtin_interfaces::msg::Duration
 */
inline builtin_interfaces::msg::Duration to_msg(std::chrono::nanoseconds duration) {
         
    builtin_interfaces::msg::Duration ret;

    // Number of nanoseconds in a seconds
    static constexpr int64_t NANOSECONDS_IN_SECOND = 1'000'000'000;

    // Parse seconds
    ret.sec = duration.count() / NANOSECONDS_IN_SECOND;
    // Parse nanoseconds remainder
    ret.nanosec = duration.count() % NANOSECONDS_IN_SECOND;
    
    return ret;
}

/* ========================================================= Ctors & dtors ======================================================== */

EthercatDriver::EthercatDriver(const rclcpp::NodeOptions & options) : 
    rclcpp::Node(NODE_NAME, options)
{

    /* ----------------------------- Initialize parameters --------------------------- */
    
    // Initialize bus config parameters
    auto eni_source            = node_common::parameters::declare_parameter_and_get(*this, ENI_SOURCE_PARAM_DESCRIPTOR);
    auto initial_drivers       = node_common::parameters::declare_parameter_and_get(*this, INITIAL_DRIVERS_PARAM_DESCRIPTOR);
    // Initialize Node thread parameters
    auto process_memory_lock             = node_common::parameters::declare_parameter_and_get(*this, PROCESS_MEMORY_LOCK_PARAM_DESCRIPTOR);
    auto process_memory_disable_trimming = node_common::parameters::declare_parameter_and_get(*this, PROCESS_MEMORY_DISABLE_TRIMMING_PARAM_DESCRIPTOR);
    auto process_memory_disable_mmap     = node_common::parameters::declare_parameter_and_get(*this, PROCESS_MEMORY_DISABLE_MMAP_PARAM_DESCRIPTOR);
    auto process_node_affinity           = node_common::parameters::declare_parameter_and_get(*this, PROCESS_NODE_AFFINITY_PARAM_DESCRIPTOR);
    auto process_node_sched_policy       = node_common::parameters::declare_parameter_and_get(*this, PROCESS_NODE_SCHED_POLICY_PARAM_DESCRIPTOR);
    auto process_node_sched_priority     = node_common::parameters::declare_parameter_and_get(*this, PROCESS_NODE_SCHED_PRIORITY_PARAM_DESCRIPTOR);
    // Initialize threads parameters
    auto cifx_log_level                         = node_common::parameters::declare_parameter_and_get(*this, CIFX_LOG_LEVEL_PARAM_DESCRIPTOR);
    auto cifx_cos_polling_interval              = node_common::parameters::declare_parameter_and_get(*this, CIFX_COS_POLLING_INTERVAL_PARAM_DESCRIPTOR);
    auto cifx_cos_polling_thread_affinity       = node_common::parameters::declare_parameter_and_get(*this, CIFX_COS_POLLING_THREAD_AFFINITY_PARAM_DESCRIPTOR);
    auto cifx_cos_polling_thread_sched_policy   = node_common::parameters::declare_parameter_and_get(*this, CIFX_COS_POLLING_THREAD_SCHED_POLICY_PARAM_DESCRIPTOR);
    auto cifx_cos_polling_thread_sched_priority = node_common::parameters::declare_parameter_and_get(*this, CIFX_COS_POLLING_THREAD_SCHED_PRIORITY_PARAM_DESCRIPTOR);
    auto cifx_irq_thread_affinity               = node_common::parameters::declare_parameter_and_get(*this, CIFX_IRQ_THREAD_AFFINITY_PARAM_DESCRIPTOR);
    auto cifx_irq_thread_sched_policy           = node_common::parameters::declare_parameter_and_get(*this, CIFX_IRQ_THREAD_SCHED_POLICY_PARAM_DESCRIPTOR);
    auto cifx_irq_thread_sched_priority         = node_common::parameters::declare_parameter_and_get(*this, CIFX_IRQ_THREAD_SCHED_PRIORITY_PARAM_DESCRIPTOR);
    
    /* ----------------------------- Validate parameters ----------------------------- */

    // Check if a valid ENI source specification given
    if(eni_source->size() != 1 and eni_source->size() != 3)
        rclcpp::exceptions::InvalidParametersException("'eni_source' must hold either one or three elements");

    // Check if a valid initial memory lock scheme given
    if(not contains(VALID_INITIAL_MEMORY_LOCKS, *process_memory_lock))
        rclcpp::exceptions::InvalidParametersException("'initial_memory_lock' must be one of { 'none', 'current', 'all' }");

    // Check if a valid CIFX log level given
    if(not contains(VALID_CIFX_LOG_LEVELS, *cifx_log_level))
        rclcpp::exceptions::InvalidParametersException("'cifx_log_level' must be one of { 'Debug', 'Info', 'Warning', 'Error' }");

    // Check if a valid CIFX IRQ polling interval given
    if(cifx_cos_polling_interval.has_value() and *cifx_cos_polling_interval <= 0)
        rclcpp::exceptions::InvalidParametersException("'cifx_polling_interval' must be positive");

    // Check if a valid scheduling policies given
    if(not contains(VALID_SCHED_POLICIES, *cifx_cos_polling_thread_sched_policy))
        rclcpp::exceptions::InvalidParametersException("'cifx_poll_thread_sched_policy' must be one of { 'SCHED_OTHER', 'SCHED_FIFO', 'SCHED_RR' }");
    if(not contains(VALID_SCHED_POLICIES, *cifx_irq_thread_sched_policy))
        rclcpp::exceptions::InvalidParametersException("'cifx_irq_thread_sched_policy' must be one of { 'SCHED_OTHER', 'SCHED_FIFO', 'SCHED_RR' }");
    if(not contains(VALID_SCHED_POLICIES, *process_node_sched_policy))
        rclcpp::exceptions::InvalidParametersException("'process_node_sched_policy' must be one of { 'SCHED_OTHER', 'SCHED_FIFO', 'SCHED_RR' }");

    /* ------------------------------ Initialize services ---------------------------- */

    /* ===== Process-config services ===== */

    *node_common::communication::make_service_builder(lock_memory_srv)
        .node(*this)
        .name(LOCK_MEMORY_SRV_TOPIC_NAME)
        .callback(*this, &EthercatDriver::lock_memory_callback);

    /* ======= Bus-control services ====== */
    
    *node_common::communication::make_service_builder(set_bus_state_srv)
        .node(*this)
        .name(SET_BUS_STATE_SRV_TOPIC_NAME)
        .callback(*this, &EthercatDriver::set_bus_state_callback);

    *node_common::communication::make_service_builder(get_bus_state_srv)
        .node(*this)
        .name(GET_BUS_STATE_SRV_TOPIC_NAME)
        .callback(*this, &EthercatDriver::get_bus_state_callback);
    
    *node_common::communication::make_service_builder(set_master_state_srv)
        .node(*this)
        .name(SET_MASTER_STATE_SRV_TOPIC_NAME)
        .callback(*this, &EthercatDriver::set_master_state_callback);

    *node_common::communication::make_service_builder(get_master_state_srv)
        .node(*this)
        .name(GET_MASTER_STATE_SRV_TOPIC_NAME)
        .callback(*this, &EthercatDriver::get_master_state_callback);

    *node_common::communication::make_service_builder(load_driver_srv)
        .node(*this)
        .name(LOAD_DRIVER_SRV_TOPIC_NAME)
        .callback(*this, &EthercatDriver::load_driver_callback);

    /* === Drivers-management services === */

    *node_common::communication::make_service_builder(unload_driver_srv)
        .node(*this)
        .name(UNLOAD_DRIVER_SRV_TOPIC_NAME)
        .callback(*this, &EthercatDriver::unload_driver_callback);

    *node_common::communication::make_service_builder(list_drivers_srv)
        .node(*this)
        .name(LIST_DRIVERS_SRV_TOPIC_NAME)
        .callback(*this, &EthercatDriver::list_drivers_callback);

    *node_common::communication::make_service_builder(get_bus_timing_srv)
        .node(*this)
        .name(GET_BUS_TIMING_SRV_TOPIC_NAME)
        .callback(*this, &EthercatDriver::get_bus_timing_callback);
        
    /* ---------------------------- Initialize private data -------------------------- */

    // Prepare driver's configuration
    EthercatDriverImpl::Config driver_config {

        // Description of the source ENI file
        .eni_source { parse_eni_config_path(*eni_source) },

        // Processing thread configuration
        .process_config {

            // Memory allocation config
            .memory_allocation {
                .lock_scheme      { parse_locking_scheme(*process_memory_lock) },
                .disable_trimming { *process_memory_disable_trimming           },
                .disable_mmap     { *process_memory_disable_mmap               }
            },

            // Thread parameters
            .processing_thread { 
                .affinity       { static_cast<unsigned>(*process_node_affinity)  },
                .sched_policy   { parse_sched_policy(*process_node_sched_policy) },
                .sched_priority { *process_node_sched_priority                   }
            }
        },

        // EtherCAT driver config
        .driver_config {

            // Log level of the driver library
            .log_level { parse_log_level(*cifx_log_level) },
            // CoS polling interval
            .cos_polling_interval { parse_polling_interval(cifx_cos_polling_interval) },
            // Configuration of the polling thread
            .cos_polling_thread_config { 
                .affinity       { static_cast<unsigned>(*cifx_cos_polling_thread_affinity)  },
                .sched_policy   { parse_sched_policy(*cifx_cos_polling_thread_sched_policy) },
                .sched_priority { *cifx_cos_polling_thread_sched_priority                   }
            },
            // Configuration of the IRQ-handling thread
            .irq_thread_config { 
                .affinity       { static_cast<unsigned>(*cifx_irq_thread_affinity)   },
                .sched_policy   { parse_sched_policy(*cifx_irq_thread_sched_policy)  },
                .sched_priority { *cifx_irq_thread_sched_priority                    }
            },
            
        }

    };

    // Construct the driver
    implementation.emplace(*this, driver_config);

    /* ---------------------------- Load initial drivers ----------------------------- */

    // If initial drivers has been requested
    if(initial_drivers.has_value() and initial_drivers->size() > 0) {

        // Iterate over requested drivers
        for(const auto &plugin_name : *initial_drivers) {

            uint64_t id;

            // Try to load driver
            try {
                id = implementation->load_driver(plugin_name);
            } catch (std::exception &e) {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to load [" << plugin_name << "] driver (" << e.what() << ")");
                continue;
            }

            // On success print message with driver's ID
            RCLCPP_INFO_STREAM(this->get_logger(), "Loaded [" << plugin_name << "] driver (ID: " << id << ")");

        }

    }

    /* --------------------------- Initialize IO waitables --------------------------- */

    // Get waitable interface interface of the node
    auto node_waitables_interface = this->get_node_waitables_interface();

    // Register read waitable with the bus-read callback
    read_waitable_trigger = std::make_shared<utilities::TriggerredWaitable<>>(std::bind(&EthercatDriver::read_io, this));
    // Register write waitable with the bus-write callback
    write_waitable_trigger = std::make_shared<utilities::TriggerredWaitable<>>(std::bind(&EthercatDriver::write_io, this));
    
    // Register waitables for monitoring
    node_waitables_interface->add_waitable(read_waitable_trigger,  nullptr);
    node_waitables_interface->add_waitable(write_waitable_trigger, nullptr);

    // Configure EtherCAT driver to trigger waitables' conditions when I/O operation is ready
    implementation->set_bus_event_handler(EthercatDriverImpl::Event::ReadBusReady, [this](){ read_waitable_trigger->trigger();  });
    implementation->set_bus_event_handler(EthercatDriverImpl::Event::WriteBusRead, [this](){ write_waitable_trigger->trigger(); });

    /* ------------------------------------------------------------------------------- */
    
    node_common::node::print_hello(*this);
}


EthercatDriver::~EthercatDriver() {
    node_common::node::print_goodbye(*this);   
}

/* =============================================== Callback methods (process config) ============================================== */

void EthercatDriver::lock_memory_callback(
    const velmwheel_ethercat_driver_msgs::srv::LockMemory::Request::SharedPtr req,
    velmwheel_ethercat_driver_msgs::srv::LockMemory::Response::SharedPtr res
) {

    MemoryLockScheme scheme;

    // Convert requested locking scheme to numerical value
    switch(req->memory_locking_scheme) {
    
        case velmwheel_ethercat_driver_msgs::srv::LockMemory::Request::CURRENT: scheme = MemoryLockScheme::Current; break;
        case velmwheel_ethercat_driver_msgs::srv::LockMemory::Request::ALL:     scheme = MemoryLockScheme::All;     break;

        // Invalid scheme
        default:

            res->success = false;
            res->error_message = "Invalid locking scheme given (" + std::to_string(req->memory_locking_scheme) + ")";
            return;
    }

    // Try to lock memory
    if(not implementation->lock_memory(scheme)) {
        res->success = false;
        res->error_message = "Failed to lock memory pages";
    // Try to disable memory trimming (if requested)
    } else if(req->disable_memory_trimming and (not implementation->disable_memory_trimming())) {
        res->success = false;
        res->error_message = "Failed to disable sbrk()";
    // Try to disable mmap (if requested)
    } else if(req->disable_mmap and (not implementation->disable_mmap())) {
        res->success = false;
        res->error_message = "Failed to disable mmap()";
    // Set response as sucesfull
    } else
        res->success = true;

    return;
}

/* ================================================ Callback methods (bus control) ================================================ */

void EthercatDriver::set_bus_state_callback(
    const velmwheel_ethercat_driver_msgs::srv::SetBusState::Request::SharedPtr req,
    velmwheel_ethercat_driver_msgs::srv::SetBusState::Response::SharedPtr res
) {

    bool bus_target_enable;

    // Convert requested locking scheme to numerical value
    switch(req->state) {
    
        case velmwheel_ethercat_driver_msgs::srv::SetBusState::Request::RUNNING: bus_target_enable = true;  break;
        case velmwheel_ethercat_driver_msgs::srv::SetBusState::Request::STOPPED: bus_target_enable = false; break;

        // Invalid scheme
        default:

            res->success = false;
            res->error_message = "Invalid target state (" + std::to_string(req->state) + ")";
            return;
    }

    // Try to set bus state
    try {
        implementation->set_bus_enabled(bus_target_enable);
    } catch (std::exception &e) {
        res->success = false;
        res->error_message = e.what();
        return;
    }

    // Fill answer 
    res->success = true;

    return;
}


void EthercatDriver::get_bus_state_callback(
    [[maybe_unused]] const velmwheel_ethercat_driver_msgs::srv::GetBusState::Request::SharedPtr req,
    velmwheel_ethercat_driver_msgs::srv::GetBusState::Response::SharedPtr res
) {

    bool bus_enabled;

    // Try to get bus state
    try {
        bus_enabled = implementation->is_bus_enabled();
    } catch (std::exception &e) {
        res->success = false;
        res->error_message = e.what();
        return;
    }

    using Response = velmwheel_ethercat_driver_msgs::srv::GetBusState::Response;

    // Fill answer 
    res->state = bus_enabled ? Response::RUNNING :Response::STOPPED;
    res->success = true;

    return;
}


void EthercatDriver::set_master_state_callback(
    const velmwheel_ethercat_driver_msgs::srv::SetMasterState::Request::SharedPtr req,
    velmwheel_ethercat_driver_msgs::srv::SetMasterState::Response::SharedPtr res
) {

    ethercat::MasterState master_target_enable;

    // Convert requested locking scheme to numerical value
    switch(req->state) {
    
        case velmwheel_ethercat_driver_msgs::srv::SetMasterState::Request::INIT:   master_target_enable = ethercat::MasterState::Init;   break;
        case velmwheel_ethercat_driver_msgs::srv::SetMasterState::Request::PREOP:  master_target_enable = ethercat::MasterState::Preop;  break;
        case velmwheel_ethercat_driver_msgs::srv::SetMasterState::Request::SAFEOP: master_target_enable = ethercat::MasterState::Safeop; break;
        case velmwheel_ethercat_driver_msgs::srv::SetMasterState::Request::OP:     master_target_enable = ethercat::MasterState::Op;     break;

        // Invalid scheme
        default:

            res->success = false;
            res->error_message = "Invalid target state (" + std::to_string(req->state) + ")";
            return;
    }

    // Try to set master state
    try {
        implementation->set_master_target_state(master_target_enable);
    } catch (std::exception &e) {
        res->success = false;
        res->error_message = e.what();
        return;
    }

    // Fill answer 
    res->success = true;

    return;
}


void EthercatDriver::get_master_state_callback(
    [[maybe_unused]] const velmwheel_ethercat_driver_msgs::srv::GetMasterState::Request::SharedPtr req,
    velmwheel_ethercat_driver_msgs::srv::GetMasterState::Response::SharedPtr res
) {

    ethercat::MasterStateInfo master_state;

    // Try to get bus state
    try {
        master_state = implementation->get_master_state();
    } catch (std::exception &e) {
        res->success = false;
        res->error_message = e.what();
        return;
    }

    using Response = velmwheel_ethercat_driver_msgs::srv::GetMasterState::Response;

    // Helper function converting master state to message constant
    auto to_msg = [](ethercat::MasterExtendedState state) {
        switch(state) {
            case ethercat::MasterExtendedState::Init:            return Response::INIT;
            case ethercat::MasterExtendedState::Preop:           return Response::PREOP;
            case ethercat::MasterExtendedState::Safeop:          return Response::SAFEOP;
            case ethercat::MasterExtendedState::Op:              return Response::OP;
            case ethercat::MasterExtendedState::Busoff:          return Response::BUSOFF;
            case ethercat::MasterExtendedState::LeaveOp:         return Response::LEAVEOP;
            case ethercat::MasterExtendedState::Busscan:         return Response::BUSSCAN;
            case ethercat::MasterExtendedState::BusscanComplete: return Response::BUSSCANCOMPLETE;
            default: /* Should not return */
                throw std::runtime_error{ 
                    "[EthercatDriver::get_master_state_callback] Unexpected master state (" 
                    + std::to_string(utilities::to_underlying(state)) + ")"
                };
        }
    };

    // Fill answer 
    res->current_state                          = to_msg(master_state.current_state);
    res->target_state                           = to_msg(master_state.target_state);
    res->stop_reason                            = master_state.stop_reason;
    res->at_least_one_mandatory_slave_not_in_op = master_state.flags.at_least_one_mandatory_slave_not_in_op;
    res->dc_xrmw_stopped                        = master_state.flags.dc_xrmw_stopped;
    res->at_least_one_mandatory_slave_lost      = master_state.flags.at_least_one_mandatory_slave_lost;
    res->success                                = true;
    
    return;
}


void EthercatDriver::get_bus_timing_callback(
    [[maybe_unused]] const velmwheel_ethercat_driver_msgs::srv::GetTimingInfo::Request::SharedPtr req,
    velmwheel_ethercat_driver_msgs::srv::GetTimingInfo::Response::SharedPtr res
) {

    ethercat::TimingInfo info;

    // Try to measure timing parameters
    try {
        info = implementation->get_timing_info();
    } catch (std::exception &e) {
        res->success = false;
        res->error_message = e.what();
        return;
    }

    // Fill answer
    res->success = true;
    res->bus_cycle              = to_msg(info.bus_cycle);
    res->frame_transmition_time = to_msg(info.frame_transmition_time);
    res->expected_bus_delay     = to_msg(info.expected_bus_delay);
    res->expected_rx_end_time   = to_msg(info.expected_rx_end_time);
    res->expected_tx_end_time   = to_msg(info.expected_tx_end_time);

    return;
}

/* =============================================== Callback methods (device drivers) ============================================== */

void EthercatDriver::load_driver_callback(
    const velmwheel_ethercat_driver_msgs::srv::LoadDriver::Request::SharedPtr req,
    velmwheel_ethercat_driver_msgs::srv::LoadDriver::Response::SharedPtr res
) {

    uint64_t id;

    // Try to load driver
    try {
        id = implementation->load_driver(req->plugin_name);
    } catch (std::exception &e) {
        res->success = false;
        res->error_message = e.what();
        return;
    }

    // Fill answer 
    res->unique_id = id;
    res->success   = true;

    return;
}


void EthercatDriver::unload_driver_callback(
    const velmwheel_ethercat_driver_msgs::srv::UnloadDriver::Request::SharedPtr req,
    velmwheel_ethercat_driver_msgs::srv::UnloadDriver::Response::SharedPtr res
) {

    // Try to unload driver
    try {
        implementation->unload_driver(req->unique_id);
    } catch (std::exception &e) {
        res->success = false;
        res->error_message = e.what();
        return;
    }

    // Fill answer
    res->success = true;

    return;
}


void EthercatDriver::list_drivers_callback(
    [[maybe_unused]] const velmwheel_ethercat_driver_msgs::srv::ListDrivers::Request::SharedPtr req,
    velmwheel_ethercat_driver_msgs::srv::ListDrivers::Response::SharedPtr res
) {
    // Get list of drivers
    auto drivers = implementation->list_drivers();

    // Reserve memory for response
    res->unique_ids.reserve(drivers.size());
    res->names.reserve(drivers.size());

    // Fill status
    res->success = true;
    // Fill response
    for(auto &driver : drivers) {
        res->unique_ids.push_back(driver.id);
        res->names.push_back(driver.plugin_name);
    }

    return;
}

/* ========================================================= I/O callbacks ======================================================== */

void EthercatDriver::read_io() {
    try {
        implementation->read_bus();
    } catch(std::exception &e) {
        RCLCPP_WARN_STREAM(this->get_logger(), "Failed to read data from the EtherCAT bus " << e.what());
    }
}


void EthercatDriver::write_io() {
    try {
        implementation->write_bus();
    } catch(std::exception &e) {
        RCLCPP_WARN_STREAM(this->get_logger(), "Failed to write data to the EtherCAT bus " << e.what());
    }
}

/* ================================================================================================================================ */

} // End namespace velmwheel

/* ======================================================== Nodes' registry ======================================================= */

RCLCPP_COMPONENTS_REGISTER_NODE(velmwheel::EthercatDriver)

/* ================================================================================================================================ */
