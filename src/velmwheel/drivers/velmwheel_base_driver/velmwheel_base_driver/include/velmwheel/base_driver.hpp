/* ============================================================================================================================ *//**
 * @file       base_driver.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 28th April 2022 9:24:14 pm
 * @modified   Monday, 18th July 2022 8:18:55 pm
 * @project    engineering-thesis
 * @brief      Definition of the driver plugin class for the servodriver EtherCAT slaves of the WUT Velmwheel robot
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __VELMWHEEL_BASE_DRIVER_H__
#define __VELMWHEEL_BASE_DRIVER_H__

/* =========================================================== Includes =========================================================== */

// Standard includes
#include <optional>
#include <vector>
// ROS includes
#include "rclcpp/visibility_control.hpp"
// Private includes
#include "velmwheel/common.hpp"
#include "velmwheel/params.hpp"
#include "velmwheel/ethercat_slave_driver.hpp"
#include "ethercat/devices/elmo.hpp"
#include "node_common.hpp"
// TF includes
#include "tf2_ros/static_transform_broadcaster.h"
// Interface includes
#include "sensor_msgs/msg/joint_state.hpp"
#include "velmwheel_msgs/msg/encoders.hpp"
#include "velmwheel_msgs/msg/encoders_stamped.hpp"
#include "velmwheel_msgs/msg/wheel.hpp"
#include "velmwheel_msgs/msg/wheel_enum.hpp"
#include "velmwheel_msgs/msg/wheels.hpp"
#include "velmwheel_base_driver_msgs/msg/wheels_status.hpp"
#include "velmwheel_base_driver_msgs/srv/enable.hpp"
#include "velmwheel_base_driver_msgs/srv/get_state.hpp"
#include "velmwheel_base_driver_msgs/srv/reset_failure.hpp"
// Private inculdes
#include "velmwheel/params.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace velmwheel {

/* ============================================================= Class ============================================================ */

/**
 * @brief Driver plugin class for the servodriver EtherCAT slaves of the WUT Velmwheel robot
 * @details At the moment, the driver provides only basic functionality that mimics behaviour 
 *    of the robot's base driver present implemented in simulation (<i>velmwheel_gazebo</i>
 *    package). The only extensions are:
 * 
 *       - @b base/status topic broadcasting status of servodrivers
 *       - @b base/reset_fault topic providing service of resetting servodrivers' faults
 *       - @b base/enable topic providing binary controll over state of the servodrives 
 *           (enabling/disabling drivers)
 *       - @b base/get_state topic providing service of getting servodrivers' state
 * 
 * @warning By default the driver is in 'Disabled' state, i.e. all Elmo drievrs are held in
 *    disabled state
 */
class RCLCPP_PUBLIC BaseDriver : public EthercatSlaveDriver {

public: /* ------------------------------------------------ Node's parameters ----------------------------------------------------- */

    /// Namespace of the IMU driver's parameters
    static constexpr auto PARAM_NAMESPACE = "base";

    /// Description of the parameter defining name of the rear left servodriver in the ENI file
    static constexpr node_common::parameters::ParamDescriptor<std::string> REAR_LEFT_WHEEL_NAME_PARAM_DESCRIPTOR {
        .name                   = "eni_wheel_names.rear_left",
        .read_only              = true,
        .dynamic_typing         = false,
        .default_value          = "WheelRearLeft",
        .description            = "Name of the rear left servodriver in the ENI file"
    };

    /// Description of the parameter defining name of the rear right servodriver in the ENI file
    static constexpr node_common::parameters::ParamDescriptor<std::string> REAR_RIGHT_WHEEL_NAME_PARAM_DESCRIPTOR {
        .name                   = "eni_wheel_names.rear_right",
        .read_only              = true,
        .dynamic_typing         = false,
        .default_value          = "WheelRearRight",
        .description            = "Name of the rear right servodriver in the ENI file"
    };

    /// Description of the parameter defining name of the front left servodriver in the ENI file
    static constexpr node_common::parameters::ParamDescriptor<std::string> FRONT_LEFT_WHEEL_NAME_PARAM_DESCRIPTOR {
        .name                   = "eni_wheel_names.front_left",
        .read_only              = true,
        .dynamic_typing         = false,
        .default_value          = "WheelRearLeft",
        .description            = "Name of the front left servodriver in the ENI file"
    };

    /// Description of the parameter defining name of the front right servodriver in the ENI file
    static constexpr node_common::parameters::ParamDescriptor<std::string> FRONT_RIGHT_WHEEL_NAME_PARAM_DESCRIPTOR {
        .name                   = "eni_wheel_names.front_right",
        .read_only              = true,
        .dynamic_typing         = false,
        .default_value          = "WheelRearRight",
        .description            = "Name of the front right servodriver in the ENI file"
    };

public: /* ------------------------------------------------ Topic's parameters ---------------------------------------------------- */

    /// Size of the topics' queue
    static constexpr std::size_t TOPIC_QUEUE_SIZE = 50;

    /// Unqualified name of the topic subscribed by the plugin to get setpoint angular velocities for robot' wheels
    static constexpr auto SETPOINT_VELOCITIES_SUB_TOPIC_NAME = "base/controls";

    /// Unqualified name of the topic used by the plugin to broadcast joints' states (repeats information on '~/encoders' topic in different form)
    static constexpr auto JOINT_STATES_PUB_TOPIC_NAME = "base/joint_states";
    /// Unqualified name of the topic used by the plugin to broadcast measurements of the robot wheels' encoders
    static constexpr auto ENCODERS_PUB_TOPIC_NAME = "base/encoders";

    /// Unqualified name of the topic broadcasting current status of the servodriver
    static constexpr auto STATUS_PUB_TOPIC_NAME = "base/status";

    /// Unqualified name of the topic providing a service of enabling/disabling servodrivers
    static constexpr auto ENABLE_SRV_TOPIC_NAME = "base/enable";
    /// Unqualified name of the topic providing a service of requesting servodrivers abotu its state
    static constexpr auto GET_STATE_SRV_TOPIC_NAME = "base/get_state";
    /// Unqualified name of the topic providing a service of resetting servodrivers' faults
    static constexpr auto RESET_FAILURE_SRV_TOPIC_NAME = "base/reset_fault";

public: /* ------------------------------------------------- Public ctors & dtors ------------------------------------------------ */

    /**
     * @brief Constructs the driver
     */
    BaseDriver() = default;

    /**
     * @brief Destructs the driver undregistering all handlers from the Master
     */
    virtual ~BaseDriver() = default;

private: /* ------------------------------------------- Private implementation API ----------------------------------------------- */

    /**
     * @brief Initialization routine of the driver. At this step implementation can register 
     *    ROS-specific interfaces like topics, subscriptions and parameters declarations
     * 
     * @param node 
     *    handle to the ROS node interface
     * @returns 
     *    function should return list of names of the slave devices that this driver wants to manage
     */
    std::vector<std::string> initialize(rclcpp::Node &node) override;

    /**
     * @brief Function called by the loading process after @ref initialize . Provides driver
     *    with handle to the interface enabling slave device management
     * 
     * @param slave 
     *    list of handles to the slave devices interfaces
     */
    void configure(std::vector<cifx::ethercat::Slave*> slaves) override;

private: /* ------------------------------------------------- Private types ------------------------------------------------------ */

    /// Helper enumeration identifying wheel drivers
    enum Wheels : std::size_t {
        RearLeft   = static_cast<std::size_t>(velmwheel::Wheel::RearLeft),
        RearRight  = static_cast<std::size_t>(velmwheel::Wheel::RearRight),
        FrontLeft  = static_cast<std::size_t>(velmwheel::Wheel::FrontLeft),
        FrontRight = static_cast<std::size_t>(velmwheel::Wheel::FrontRight),
        Num        = static_cast<std::size_t>(velmwheel::params::WHEEL_NUM)
    };

    /**
     * @brief Polarities of robot's wheels. Left wheels have their polarities reversed
     */
    static constexpr std::array<double, Wheels::Num> WheelsPolarities {
        /* RearLeft   */ -1,
        /* RearRight  */  1,
        /* FrontLeft  */ -1,
        /* FrontRight */  1
    };

private: /* ---------------------------------------------- Private ROS callbacks ------------------------------------------------- */
    
    /**
     * @brief Callback receiving incoming wheel controls
     * @note Wheel's controlls are interpreted as target speeds in [rad/s]
     */
    void setpoint_velocities_callback(const velmwheel_msgs::msg::Wheels &msg);

    /**
     * @brief Creates a functor handling incoming-data for the given wheel
     * 
     * @param wheel 
     *    index of the wheel to be handled
     */
    std::function<void(void)> make_driver_callback(Wheels wheel);

    /**
     * @brief Callback triggerred when new data form all drivers is received by the driver.
     *    In sucha a case, callback forms output messages and pushes them to the ROS 
     *    infrstructure
     */
    void broadcast_callback();

    /**
     * @brief Callback managing service of enabling/disabling servodrivers
     */
    void enable_callback(
        const velmwheel_base_driver_msgs::srv::Enable::Request::SharedPtr req,
        velmwheel_base_driver_msgs::srv::Enable::Response::SharedPtr res
    );

    /**
     * @brief Callback managing service of requesting servodrivers about it's state
     */
    void get_state_callback(
        const velmwheel_base_driver_msgs::srv::GetState::Request::SharedPtr req,
        velmwheel_base_driver_msgs::srv::GetState::Response::SharedPtr res
    );
    
    /**
     * @brief Callback managing service of resetting servodrivers' faults
     */
    void reset_failure_callback(
        const velmwheel_base_driver_msgs::srv::ResetFailure::Request::SharedPtr req,
        velmwheel_base_driver_msgs::srv::ResetFailure::Response::SharedPtr res
    );

private: /* --------------------------------------------- Private ROS interfaces ------------------------------------------------- */

    /// Subscriber topic subscribed by the plugin to get setpoint angular velocities for robot' wheels
    rclcpp::Subscription<velmwheel_msgs::msg::Wheels>::SharedPtr setpoint_velocities_sub;

    /// Publisher topic used by the plugin to broadcast joints' states (repeats information on '~/encoders' topic in different form)
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub;
    /// Publisher topic used by the plugin to broadcast measurements of the robot wheels' encoders
    rclcpp::Publisher<velmwheel_msgs::msg::EncodersStamped>::SharedPtr encoders_pub;

    /// Publisher topic broadcasting current status of the servodriver
    rclcpp::Publisher<velmwheel_base_driver_msgs::msg::WheelsStatus>::SharedPtr status_pub;

    /// Service topic providing a service of enabling/disabling servodrivers
    rclcpp::Service<velmwheel_base_driver_msgs::srv::Enable>::SharedPtr enable_srv;
    /// Service topic providing a service of requesting servodrivers about its state
    rclcpp::Service<velmwheel_base_driver_msgs::srv::GetState>::SharedPtr get_state;
    /// Service topic providing a service of resetting servodrivers' faults
    rclcpp::Service<velmwheel_base_driver_msgs::srv::ResetFailure>::SharedPtr reset_failure_srv;
    
    /// TF2 publishing object for broadcastign transformation from <base_link> to <velmwheel> frame
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster;

private: /* ------------------------------------------------- Private types ------------------------------------------------------ */

    /// Type of the driver class used by the node
    using Driver = ethercat::devices::Elmo<cifx::ethercat::Slave>;

    /**
     * @brief Structure describing current measurements/status informations from a single
     *    servodriver
     */
    struct WheelMeasurements {

        /// Current position of the wheel in [rad]
        double position;
        /// Current velocity of the wheel in [rad/s]
        double velocity;
        /// Current status of the servodriver
        Driver::State state { Driver::State::NotReadyToSwitchOn };

    };

    /**
     * @brief Structure holding current measurements/status informations from all servodrivers
     */
    struct WheelsMeasurementsSet {

        /// Data associated with subsequent servos
        std::array<WheelMeasurements, Wheels::Num> data;

        /**
         * @brief Counter of @a data elements that has been updated since last messages published
         *    on ROS topics.
         * @details Each slave driver has it's own 'calback' method registered that is called each
         *    time new data arrives from the bus. In this callback, each driver updates it's own
         *    @a data element and increments @a update_counter . Data are compiled and pushed into
         *    the ROSsystem each time callback sees that @a update_counter meets (counter % 4 = 0) 
         *    condition.
         * 
         * @note <i>ethercatlib</i> guarantees that now two slave callbacks will be called at the 
         *    same time to no synchronisation of the @a update_counter is required
         */
        std::size_t update_counter;

    };

private: /* -------------------------------------------------- Private data ------------------------------------------------------ */

    /// Handle to the ROS node
    rclcpp::Node *node { nullptr };
    /// Drivers implementation
    std::vector<Driver> drivers;

    /// Measurements
    WheelsMeasurementsSet measurements;
    /// Counter of initialized measurements skipped before sending first meassage to the ROS system
    std::size_t initial_measurements_skip_counter { 10 };
    
    /// Timeout for all drivers to recover from fault after receiving fault-reset signal
    static constexpr std::chrono::milliseconds FaultRecoveryTimeout{ 10'000 };

    /**
     * @brief Enumeration indicating current target state of the driver
     * @details This variable provides quasi 'state machine' to the working principle of the driver.
     *    By default driver is inactive - it keeps all servos enabled but in no-operational state.
     *    If external source triggers enable signal, the driver will switch to 'Active' state in which
     *    it willl try to move all servos into the operation state [1]. If at any time, any wheel reports
     *    a fault condition, driver will switch to 'Fault' state in which all wheels will be deactivated.
     *    This working principle can be described with use of a simple FSM diagram
     *     
     *    @startuml
     * 
     *    [*]        --> Inactive
     *    Inactive   --> Active : enable request
     *    Inactive   --> Fault : fault condition\n(should nto happen)
     *    Active     --> Fault : fault condition
     *    Active     --> Inactive : disable request
     *    Fault      --> Recovering : fault reset\nrequest
     *    Recovering --> Fault : recovery\ntimeout
     *    Recovering --> Inactive : servos\nrecovered
     * 
     *    @enduml
     * 
     * @see [1] See Elmo servo state machin in the documentation
     */
    enum class State {
        Inactive,
        Active,
        Fault,
        Recovering
    } state = State::Inactive;

    // Timestamp of thre recover timeout start
    rclcpp::Time recovery_start;

    /**
     * @brief 4-bit bitset indicating incoming 'Reset Fault' request from the external sourcer
     * @details For the same reason as in case of @ref WheelsMeasurementsSet::update_counter , the fault
     *    reset flag needs to be divided in per-wheel flags set. All servodrivers are (in some sense)
     *    autonomous entities. As so they need to handle their fault conditions independently. However
     *    state of the whole driver ( @ref state ) is shared by all servodrivers. For this read, this request
     *    flag has been divided into 4 intependent flags.
     * 
     *    When driver receives request from the external source it sets this bitset to @c 0b1111 (as long
     *    as the driver is currently in the 'Fault' state). On the next bus iteration each driver tests
     *    it's flag and (if the driver is actually in the fault state) resets it's own fault. Next it resets
     *    corresponding flag in the bitset. The last servodriver to see this request (i.e. the one that 
     *    resets the whole bitset to @c 0b0000 state) switches @ref state of the whole driver to 'Inactive'
     * 
     * @todo If FSM describing the driver becomes any bigger, use boost-sml to describe it's behaviour
     */
    std::bitset<Wheels::Num> fault_reset_request;

    /// Auxiliary bitset denting what drivers have already left fault state
    std::bitset<Wheels::Num> fault_recovery_status;

};

/* ================================================================================================================================ */

} // End namespace velmwheel

#endif
