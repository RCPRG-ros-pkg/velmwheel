/* ============================================================================================================================ *//**
 * @file       ethercat_driver.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 28th April 2022 4:00:25 pm
 * @modified   Tuesday, 31st May 2022 2:00:01 pm
 * @project    engineering-thesis
 * @brief      Definition of the class implementing bus-driving velmwheel_ethercat_driver Node
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __VELMWHEEL_ETHERCAT_DRIVER_H__
#define __VELMWHEEL_ETHERCAT_DRIVER_H__

/* =========================================================== Includes =========================================================== */

// System includes
#include <optional>
// ROS includes
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rclcpp_components/register_node_macro.hpp"
// Private interface includes
#include "velmwheel_ethercat_driver_msgs/srv/lock_memory.hpp"
#include "velmwheel_ethercat_driver_msgs/srv/get_bus_state.hpp"
#include "velmwheel_ethercat_driver_msgs/srv/set_bus_state.hpp"
#include "velmwheel_ethercat_driver_msgs/srv/get_master_state.hpp"
#include "velmwheel_ethercat_driver_msgs/srv/set_master_state.hpp"
#include "velmwheel_ethercat_driver_msgs/srv/load_driver.hpp"
#include "velmwheel_ethercat_driver_msgs/srv/unload_driver.hpp"
#include "velmwheel_ethercat_driver_msgs/srv/list_drivers.hpp"
#include "velmwheel_ethercat_driver_msgs/srv/get_timing_info.hpp"
// Private includes
#include "node_common/parameters.hpp"
#include "node_common/communication.hpp"
#include "velmwheel/ethercat_driver_impl.hpp"
#include "velmwheel/utilities/triggerred_waitable.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace velmwheel {

/* ======================================================== Auxiliary types ======================================================= */

/**
 * @class EthercatDriver
 * @brief Class implementing bus-driving velmwheel_ethercat_driver Node
 */
class RCLCPP_PUBLIC EthercatDriver : public rclcpp::Node {

public: /* -------------------------------------------------- Node's traits ------------------------------------------------------- */

    /// Name of the node
    static constexpr auto NODE_NAME = "ethercat_driver";

public: /* ------------------------------------------ Node's parameters (bus config) ---------------------------------------------- */
    
    /// Number of elements in the vector initializing ENI resource description
    static constexpr std::size_t ENI_SOURCE_INIT_SIZE = 3;

    /// Description of the parameter defining source ENI configuration
    static constexpr node_common::parameters::ParamDescriptor<std::vector<std::string>, ENI_SOURCE_INIT_SIZE> ENI_SOURCE_PARAM_DESCRIPTOR {
        .name           = "eni_source",
        .read_only      = true,
        .dynamic_typing = false,
        .default_value  = std::array{ "velmwheel_twincat", "eni", "default_config.xml" },
        .description    = "Description of the ENI configuration source. This shall be given as:            \n"
                          "                                                                                \n"
                          "  1) 3-element touple holding                                                   \n"
                          "                                                                                \n"
                          "         - source package                                                       \n"
                          "         - source resource                                                      \n"
                          "         - source file                                                          \n"
                          "                                                                                \n"
                          "      In such a case node assumes that the @p package registers ament           \n"
                          "      resource named @p resource whose content is a path to the directory       \n"
                          "      containing ENI files (relative to the @package prefix). The ENI file      \n"
                          "      named @p file is expected to reside in this directory.                    \n"
                          "                                                                                \n"
                          "  2) 1-element touple describing                                                \n"
                          "                                                                                \n"
                          "         - absolute path to the ENI file                                        \n"
                          "                                                                                \n"
                          "     In such a case node loads ENI file from the given path                     \n"
                          "                                                                                \n"
                          "Given ENI file is not cached in the node's memory. It is reread every time the  \n"
                          "node need to access ENI data (e.g. when loading slave driver). Location of the  \n"
                          "file must be valid for the entire node's lifetime                                 "
    };

    /// Description of the parameter defining list of driver plugins that should be loaded on construction
    static constexpr node_common::parameters::ParamDescriptor<std::vector<std::string>> INITIAL_DRIVERS_PARAM_DESCRIPTOR {
        .name                   = "initial_drivers",
        .read_only              = true,
        .dynamic_typing         = false,
        .description            = "List of driver plugins that should be loaded on construction"
    };

public: /* ------------------------------------------------ Process parameters ---------------------------------------------------- */

    /// Description of the parameter defining initial memory locking scheme
    static constexpr node_common::parameters::ParamDescriptor<std::string> PROCESS_MEMORY_LOCK_PARAM_DESCRIPTOR {
        .name                   = "process.memory.lock",
        .read_only              = true,
        .dynamic_typing         = false,
        .default_value          = "none",
        .description            = "Initial memory pages locking scheme of the driver process When 'all' given "
                                  "current as well as future memory pages of the node will be locked.",
        .additional_constraints = "One of { 'none', 'current', 'all' }"
    };

    /// Description of the parameter defining whether to diable allocated memory trimming at node startup
    static constexpr node_common::parameters::ParamDescriptor<bool> PROCESS_MEMORY_DISABLE_TRIMMING_PARAM_DESCRIPTOR {
        .name                   = "process.memory.disable_trimming",
        .read_only              = true,
        .dynamic_typing         = false,
        .default_value          = false,
        .description            = "If 'true' node will disable trimming memory with sbrk() at startup"
    };

    /// Description of the parameter defining whether to diable mmap() usage for malloc() at stratup
    static constexpr node_common::parameters::ParamDescriptor<bool> PROCESS_MEMORY_DISABLE_MMAP_PARAM_DESCRIPTOR {
        .name                   = "process.memory.disable_mmap",
        .read_only              = true,
        .dynamic_typing         = false,
        .default_value          = false,
        .description            = "If 'true' node will disable mmap() usage ats tartup"
    };

    /// Description of the parameter defining affinity of the IRQ-polling CIFX Toolkit thread
    static constexpr node_common::parameters::ParamDescriptor<int> PROCESS_NODE_AFFINITY_PARAM_DESCRIPTOR {
        .name           = "process.node.affinity",
        .read_only      = true,
        .dynamic_typing = false,
        .default_value  = 0xF,
        .description    = "Affinity of the node thread",
    };

    /// Description of the parameter defining scheduling policy of the IRQ-polling CIFX Toolkit thread
    static constexpr node_common::parameters::ParamDescriptor<std::string> PROCESS_NODE_SCHED_POLICY_PARAM_DESCRIPTOR {
        .name                   = "process.node.sched_policy",
        .read_only              = true,
        .dynamic_typing         = false,
        .default_value          = "SCHED_OTHER",
        .description            = "Scheduling policy of the node thread",
        .additional_constraints = "One of { 'SCHED_OTHER', 'SCHED_FIFO', 'SCHED_RR' }"
    };

    /// Description of the parameter defining scheduling priority of the IRQ-polling CIFX Toolkit thread
    static constexpr node_common::parameters::ParamDescriptor<int> PROCESS_NODE_SCHED_PRIORITY_PARAM_DESCRIPTOR {
        .name           = "process.node.sched_priority",
        .read_only      = true,
        .dynamic_typing = false,
        .default_value  = 0,
        .description    = "Scheduling priority of the node thread",
    };

public: /* ------------------------------------------- Node's parameters (threads) ------------------------------------------------ */

    /// Description of the parameter defining log level of the CIFX toolkit
    static constexpr node_common::parameters::ParamDescriptor<std::string> CIFX_LOG_LEVEL_PARAM_DESCRIPTOR {
        .name                   = "cifx.log_level",
        .read_only              = true,
        .dynamic_typing         = false,
        .default_value          = "Info",
        .description            = "Log level of the CIFX toolkit",
        .additional_constraints = "One of { 'Debug', 'Info', 'Warning', 'Error' }"
    };

    /// Description of the parameter defining CoS (Change of State) polling interval of CIFX toolkit
    static constexpr node_common::parameters::ParamDescriptor<double> CIFX_COS_POLLING_INTERVAL_PARAM_DESCRIPTOR {
        .name           = "cifx.cos_polling_interval",
        .read_only      = true,
        .dynamic_typing = false,
        .description    = "CoS polling interval of CIFX toolkit in [s] (polling not used when parameter not defined)",
    };

    /// Description of the parameter defining affinity of the CoS (Change of State) polling CIFX Toolkit thread
    static constexpr node_common::parameters::ParamDescriptor<int> CIFX_COS_POLLING_THREAD_AFFINITY_PARAM_DESCRIPTOR {
        .name           = "cifx.cos_polling_thread.affinity",
        .read_only      = true,
        .dynamic_typing = false,
        .default_value  = 0xF,
        .description    = "Affinity of the CoS-polling CIFX Toolkit thread",
    };

    /// Description of the parameter defining scheduling policy of the CoS (Change of State) polling CIFX Toolkit thread
    static constexpr node_common::parameters::ParamDescriptor<std::string> CIFX_COS_POLLING_THREAD_SCHED_POLICY_PARAM_DESCRIPTOR {
        .name                   = "cifx.cos_polling_thread.sched_policy",
        .read_only              = true,
        .dynamic_typing         = false,
        .default_value          = "SCHED_OTHER",
        .description            = "Scheduling policy of the IRQ-polling CIFX Toolkit thread",
        .additional_constraints = "One of { 'SCHED_OTHER', 'SCHED_FIFO', 'SCHED_RR' }"
    };

    /// Description of the parameter defining scheduling priority of the CoS (Change of State) polling CIFX Toolkit thread
    static constexpr node_common::parameters::ParamDescriptor<int> CIFX_COS_POLLING_THREAD_SCHED_PRIORITY_PARAM_DESCRIPTOR {
        .name           = "cifx.cos_polling_thread.sched_priority",
        .read_only      = true,
        .dynamic_typing = false,
        .default_value  = 0,
        .description    = "Scheduling priority of the IRQ-polling CIFX Toolkit thread",
    };

    /// Description of the parameter defining affinity of the IRQ-polling CIFX Toolkit thread
    static constexpr node_common::parameters::ParamDescriptor<int> CIFX_IRQ_THREAD_AFFINITY_PARAM_DESCRIPTOR {
        .name           = "cifx.irq_thread.affinity",
        .read_only      = true,
        .dynamic_typing = false,
        .default_value  = 0xF,
        .description    = "Affinity of the IRQ-handling CIFX Toolkit thread",
    };

    /// Description of the parameter defining scheduling policy of the IRQ-polling CIFX Toolkit thread
    static constexpr node_common::parameters::ParamDescriptor<std::string> CIFX_IRQ_THREAD_SCHED_POLICY_PARAM_DESCRIPTOR {
        .name                   = "cifx.irq_thread.sched_policy",
        .read_only              = true,
        .dynamic_typing         = false,
        .default_value          = "SCHED_OTHER",
        .description            = "Scheduling policy of the IRQ-handling CIFX Toolkit thread",
        .additional_constraints = "One of { 'SCHED_OTHER', 'SCHED_FIFO', 'SCHED_RR' }"
    };

    /// Description of the parameter defining scheduling priority of the IRQ-polling CIFX Toolkit thread
    static constexpr node_common::parameters::ParamDescriptor<int> CIFX_IRQ_THREAD_SCHED_PRIORITY_PARAM_DESCRIPTOR {
        .name           = "cifx.irq_thread.sched_priority",
        .read_only      = true,
        .dynamic_typing = false,
        .default_value  = 0,
        .description    = "Scheduling priority of the IRQ-handling CIFX Toolkit thread",
    };

public: /* ------------------------------------------------ Topic's parameters ---------------------------------------------------- */

    // Size of the topics' queue
    static constexpr std::size_t TOPIC_QUEUE_SIZE = 50;

    // Unqualified name of the service shared by the driver to lock driver memory
    static constexpr auto LOCK_MEMORY_SRV_TOPIC_NAME = "~/lock_memory";

    // Unqualified name of the service shared by the driver to set EtherCAT bus state
    static constexpr auto SET_BUS_STATE_SRV_TOPIC_NAME = "~/set_bus_state";
    // Unqualified name of the service shared by the driver to obtain EtherCAT bus state
    static constexpr auto GET_BUS_STATE_SRV_TOPIC_NAME = "~/get_bus_state";
    // Unqualified name of the service shared by the driver to set target EtherCAT master state
    static constexpr auto SET_MASTER_STATE_SRV_TOPIC_NAME = "~/set_master_state";
    // Unqualified name of the service shared by the driver to obtain EtherCAT master state
    static constexpr auto GET_MASTER_STATE_SRV_TOPIC_NAME = "~/get_master_state";
    // Unqualified name of the service shared by the driver to measure EtherCAT bus timing parameters
    static constexpr auto GET_BUS_TIMING_SRV_TOPIC_NAME = "~/get_bus_timing";

    // Unqualified name of the service shared by the driver to load slave driver plugin
    static constexpr auto LOAD_DRIVER_SRV_TOPIC_NAME = "~/load_driver";
    // Unqualified name of the service shared by the driver to list currenyl loaded slave drivers
    static constexpr auto LIST_DRIVERS_SRV_TOPIC_NAME = "~/list_drivers";
    // Unqualified name of the service shared by the driver to unload slave driver plugin
    static constexpr auto UNLOAD_DRIVER_SRV_TOPIC_NAME = "~/unload_driver";

public: /* ----------------------------------------------- Public ctors & dtors --------------------------------------------------- */

    /**
     * @brief Construct a new EthercatDriver node
     * 
     * @param options 
     *    configuration of the node
     */
    EthercatDriver(const rclcpp::NodeOptions & options);

    /**
     * @brief Destructs EthercatDriver node
     */
    ~EthercatDriver();

private: /* ---------------------------------------- Callback methods (process config) -------------------------------------------- */

    /**
     * @brief Callback method to the ROS2 service providing memory locking of the driver process
     * 
     * @param req 
     *    request structure containing locking scheme
     * @param res 
     *    reference to the response structure
     */
    void lock_memory_callback(
        const velmwheel_ethercat_driver_msgs::srv::LockMemory::Request::SharedPtr req,
        velmwheel_ethercat_driver_msgs::srv::LockMemory::Response::SharedPtr res
    );

private: /* ------------------------------------------ Callback methods (bus control) --------------------------------------------- */

    /**
     * @brief Callback method to the ROS2 service setting state of the EtherCAT bus
     * 
     * @param req 
     *    request structure containing target state
     * @param res 
     *    reference to the response structure
     */
    void set_bus_state_callback(
        const velmwheel_ethercat_driver_msgs::srv::SetBusState::Request::SharedPtr req,
        velmwheel_ethercat_driver_msgs::srv::SetBusState::Response::SharedPtr res
    );

    /**
     * @brief Callback method to the ROS2 service reading state of the EtherCAT bus
     * 
     * @param req 
     *    request structure [unused]
     * @param res 
     *    reference to the response structure
     */
    void get_bus_state_callback(
        const velmwheel_ethercat_driver_msgs::srv::GetBusState::Request::SharedPtr req,
        velmwheel_ethercat_driver_msgs::srv::GetBusState::Response::SharedPtr res
    );

    /**
     * @brief Callback method to the ROS2 service setting target state of the EtherCAT master
     * 
     * @param req 
     *    request structure containing target state
     * @param res 
     *    reference to the response structure
     */
    void set_master_state_callback(
        const velmwheel_ethercat_driver_msgs::srv::SetMasterState::Request::SharedPtr req,
        velmwheel_ethercat_driver_msgs::srv::SetMasterState::Response::SharedPtr res
    );

    /**
     * @brief Callback method to the ROS2 service reading state of the EtherCAT master
     * 
     * @param req 
     *    request structure [unused]
     * @param res 
     *    reference to the response structure
     */
    void get_master_state_callback(
        const velmwheel_ethercat_driver_msgs::srv::GetMasterState::Request::SharedPtr req,
        velmwheel_ethercat_driver_msgs::srv::GetMasterState::Response::SharedPtr res
    );

    /**
     * @brief Callback method to the ROS2 service requesting measurement of bus timing parameters
     * 
     * @param req 
     *    request structure [unused]
     * @param res 
     *    reference to the response structure
     */
    void get_bus_timing_callback(
        const velmwheel_ethercat_driver_msgs::srv::GetTimingInfo::Request::SharedPtr req,
        velmwheel_ethercat_driver_msgs::srv::GetTimingInfo::Response::SharedPtr res
    );

private: /* ----------------------------------------- Callback methods (device drivers) ------------------------------------------- */

    /**
     * @brief Callback method to the ROS2 service loading driver plugin for the EtherCAT slave device(s)
     * 
     * @param req 
     *    request structure containing specification of the target driver
     * @param res 
     *    reference to the response structure
     */
    void load_driver_callback(
        const velmwheel_ethercat_driver_msgs::srv::LoadDriver::Request::SharedPtr req,
        velmwheel_ethercat_driver_msgs::srv::LoadDriver::Response::SharedPtr res
    );

    /**
     * @brief Callback method to the ROS2 service unloading driver plugin of the EtherCAT slave device(s)
     * 
     * @param req 
     *    request structure containing specification of the target driver
     * @param res 
     *    reference to the response structure
     */
    void unload_driver_callback(
        const velmwheel_ethercat_driver_msgs::srv::UnloadDriver::Request::SharedPtr req,
        velmwheel_ethercat_driver_msgs::srv::UnloadDriver::Response::SharedPtr res
    );

    /**
     * @brief Callback method to the ROS2 service listing currently loaded drivers
     * 
     * @param req 
     *    request structure [unused]
     * @param res 
     *    reference to the response structure
     */
    void list_drivers_callback(
        const velmwheel_ethercat_driver_msgs::srv::ListDrivers::Request::SharedPtr req,
        velmwheel_ethercat_driver_msgs::srv::ListDrivers::Response::SharedPtr res
    );

private: /* -------------------------------------------------- I/O Callbacks ------------------------------------------------------ */

    /**
     * @brief Reads data incoming from the bus
     */
    void read_io();

    /**
     * @brief Collects and writes data to the bus
     */
    void write_io();

private: /* ------------------------------------------------- ROS interfaces ------------------------------------------------------ */

	/// Service interface used to lock driver memory
	rclcpp::Service<velmwheel_ethercat_driver_msgs::srv::LockMemory>::SharedPtr lock_memory_srv;

	/// Service interface used to set EtherCAT bus state
	rclcpp::Service<velmwheel_ethercat_driver_msgs::srv::SetBusState>::SharedPtr set_bus_state_srv;
	/// Service interface used to obtain EtherCAT bus state
	rclcpp::Service<velmwheel_ethercat_driver_msgs::srv::GetBusState>::SharedPtr get_bus_state_srv;
	/// Service interface used to set EtherCAT master target state
	rclcpp::Service<velmwheel_ethercat_driver_msgs::srv::SetMasterState>::SharedPtr set_master_state_srv;
	/// Service interface used to obtain EtherCAT master state
	rclcpp::Service<velmwheel_ethercat_driver_msgs::srv::GetMasterState>::SharedPtr get_master_state_srv;
	/// Service interface used to measure EtherCAT bus timing parameters
	rclcpp::Service<velmwheel_ethercat_driver_msgs::srv::GetTimingInfo>::SharedPtr get_bus_timing_srv;

	/// Service interface used to load slave driver plugin
	rclcpp::Service<velmwheel_ethercat_driver_msgs::srv::LoadDriver>::SharedPtr load_driver_srv;
	/// Service interface used to unload slave driver plugin
	rclcpp::Service<velmwheel_ethercat_driver_msgs::srv::UnloadDriver>::SharedPtr unload_driver_srv;
	/// Service interface used to list currently loaded drivers slave driver plugin
	rclcpp::Service<velmwheel_ethercat_driver_msgs::srv::ListDrivers>::SharedPtr list_drivers_srv;

    /// Waitable interface triggering read I/O operation
    std::shared_ptr<utilities::TriggerredWaitable<>> read_waitable_trigger;
    /// Waitable interface triggering write I/O operation
    std::shared_ptr<utilities::TriggerredWaitable<>> write_waitable_trigger;

private: /* -------------------------------------------------- Node's state ------------------------------------------------------- */

    // Implementation of the driver (defined as optional to defer construction to the constructor's body)
    std::optional<EthercatDriverImpl> implementation;

};

/* ================================================================================================================================ */

} // End namespace velmwheel

#endif
