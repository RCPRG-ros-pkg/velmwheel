/** ==================================================================================================================================
 * @file       triggerred_waitable.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 28th April 2022 3:13:39 pm
 * @modified   Friday, 20th May 2022 3:06:46 pm
 * @project    engineering-thesis
 * @brief
 *    
 *    Definition of the custom rclcpp::Waitable implementation providing a synchronization mechanism between ROS executor and
 *    external events source (i.e. other threads)
 *    
 * @soruce https://answers.ros.org/question/322815/ros2-how-to-create-custom-waitable/
 * @see https://docs.ros.org/en/galactic/Concepts/About-Executors.html
 * @see https://discourse.ros.org/t/proper-way-to-write-a-driver-component-compatible/11809
 *    
 * @copyright Krzysztof Pierczyk © 2022
 * ================================================================================================================================ */

#ifndef __VELMWHEEL_UTILITIES_TRIGGERRED_WAITABLE_H__
#define __VELMWHEEL_UTILITIES_TRIGGERRED_WAITABLE_H__

/* =========================================================== Includes =========================================================== */

// System includes
#include <atomic>
#include <chrono>
#include <mutex>
// ROS includes
#include "rclcpp/rclcpp.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace velmwheel {
namespace utilities {

/* ========================================================== Definitions ========================================================= */

/**
 * @brief Implementation of custom rclcpp::Waitable interface providing a way to dispatch node's callback
 *    when the external (potentially no-ros-related) event occurs
 * 
 * @tparam RecursiveLock
 *    recursive lock type used to synchronise calls to @ref add_to_wait_set(...) between execution threads
 * 
 * @note In the official ROS2 documentation there are no informations about usage of custom rclcpp::Waitable
 *    interfaces. There is only a tiny mention about their existance [1]. This implementation bases on the
 *    [2] example, set of empirical tests ( @see spike/custom_waitable ) and a lot of suppositions. As so
 *    it may turn out in the future that an improper threads-synchronisation mechanism has been implemented.
 *    For now, no tests shown race conditions with the use of this implementation.
 * 
 * @see [1] https://github.com/ros2/ros2_documentation/blob/rolling/source/Releases/Release-Galactic-Geochelone.rst
 * @see [2] https://answers.ros.org/question/322815/ros2-how-to-create-custom-waitable/
 * 
 * @code 
 * 
 *    // Create node
 *    auto node = std::make_shared<rclcpp::Node>(...);
 *    // Get waitable interface
 *    auto node_waitables_interface = node->get_node_waitables_interface();
 *    // Create waitable implementation
 *    auto triggerred_waitable = std::make_shared<TriggerredWaitable<>>(some_callback);
 *    // Add waitable to the node's interface
 *    node_waitables_interface->add_waitable(triggerred_waitable, // callback_group // nullptr);
 * 
 *    // Execute the node
 *    rclcpp::spin(node);
 * 
 *    ... 
 * 
 *    (in other thread)
 * 
 *    // Trigger the event
 *    triggerred_waitable->trigger();
 * 
 * @endcode
 */
template<typename RecursiveLock = std::recursive_mutex>
class TriggerredWaitable : public rclcpp::Waitable {

public: /* ----------------------------------------------- Waitable API iplementation --------------------------------------------- */
    
    /**
     * @brief Initializes waitable with a single @ref rcl_guard_condition_options_t 
     * 
     * @throws std::runtime_error 
     *    if constructor could not initialize guard condition structure
     */
    TriggerredWaitable(std::function<void(void)> callback);

    /**
     * @brief Gives a hint to the Waitable API about number of associated guard conditions
     */
    inline size_t get_number_of_ready_guard_conditions() override;

    /**
     * @brief Adds waitable to the wait set of the executing thread
     * 
     * @param[inout] wait_set 
     *    wait set to add waitable to
     * @returns 
     *    @retval @c true on success
     *    @retval @c false on failure
     */
    bool add_to_wait_set(rcl_wait_set_t * wait_set) override;

    /**
     * @brief Tells Waitable API whether the associated condition has been triggerred
     * 
     * @param wait_set 
     *    wait set that the waitable has been added to
     * @return 
     *    @retval @c true if waitable is ready to execute
     *    @retval @c false otherwise
     */
    bool is_ready(rcl_wait_set_t * wait_set) override;

    /**
     * @brief Takes data related to the waited conditon
     * @unused 
     */
    std::shared_ptr<void> take_data() override;

    /**
     * @brief Actual routine executed by the execution thread
     */
    void execute(std::shared_ptr<void> & data) override;

public: /* ------------------------------------------------------- Client API ----------------------------------------------------- */

    /**
     * @brief Triggers the event
     */
    void trigger();

private: /* ----------------------------------------------------- Data members ---------------------------------------------------- */

    /// Synchronisation lock
    RecursiveLock lock;
    
    /// Guarding condition
    rcl_guard_condition_t gc { rcl_get_zero_initialized_guard_condition() };
    
    /// External event state
    std::atomic<bool> triggerred { ATOMIC_FLAG_INIT };

    /// Associated callback
    std::function<void(void)> callback;

};

/* ================================================================================================================================ */

} // End namespace utilities
} // End namespace velmwheel

/* ==================================================== Implementation includes =================================================== */

#include "velmwheel/utilities/triggerred_waitable/triggerred_waitable.hpp"

/* ================================================================================================================================ */

#endif
