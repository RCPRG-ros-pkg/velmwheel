/* ============================================================================================================================ *//**
 * @file       triggerred_waitable.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 28th April 2022 3:13:39 pm
 * @modified   Thursday, 28th April 2022 3:58:28 pm
 * @project    engineering-thesis
 * @brief      Definition of the methods of the custom rclcpp::Waitable implementation providing a synchronization mechanism between ROS 
 *             executor and external events source (i.e. other threads)
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __VELMWHEEL_UTILITIES_TRIGGERRED_WAITABLE_TRIGGERRED_WAITABLE_H__
#define __VELMWHEEL_UTILITIES_TRIGGERRED_WAITABLE_TRIGGERRED_WAITABLE_H__

/* =========================================================== Includes =========================================================== */

// Private includes
#include "velmwheel/utilities/triggerred_waitable.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace velmwheel {
namespace utilities {

/* ================================================== Waitable API iplementation ================================================== */

template<typename RecursiveLock>
TriggerredWaitable<RecursiveLock>::TriggerredWaitable(
    std::function<void(void)> callback
) : 
    callback{ callback } 
{

    // Get global context
    std::shared_ptr<rclcpp::Context> context_ptr =  rclcpp::contexts::get_global_default_context();
    
    // Create options for guard condition
    rcl_guard_condition_options_t guard_condition_options = rcl_guard_condition_get_default_options();
    // Initialize guard condition
    rcl_ret_t ret = rcl_guard_condition_init(&gc, context_ptr->get_rcl_context().get(), guard_condition_options);
    // If initialization failed, throw error
    if(ret != RCL_RET_OK)
        throw std::runtime_error{ "[TriggerredWaitable] rcl_guard_condition_init failed to initialize guard condition" };

}


template<typename RecursiveLock>
size_t TriggerredWaitable<RecursiveLock>::get_number_of_ready_guard_conditions() { return 1; }


template<typename RecursiveLock>
bool TriggerredWaitable<RecursiveLock>::add_to_wait_set(rcl_wait_set_t * wait_set) {

    std::lock_guard<std::recursive_mutex> guard(lock);
    
    // Add condition to the set
    rcl_ret_t ret = rcl_wait_set_add_guard_condition(wait_set, &gc, NULL);
    // return adding result
    return RCL_RET_OK == ret;

}


template<typename RecursiveLock>
bool TriggerredWaitable<RecursiveLock>::is_ready([[maybe_unused]] rcl_wait_set_t * wait_set) { 

    // Expected state of the event (triggerred)
    bool expected = true;
    // Desired state of the event after consumption
    bool desired = false;

    /**
     * @todo Set proper memory order model for the compare-exchange call
     */

    return triggerred.compare_exchange_strong(expected, desired);
}


template<typename RecursiveLock>
std::shared_ptr<void> TriggerredWaitable<RecursiveLock>::take_data() { 
    return nullptr; 
}


template<typename RecursiveLock>
void TriggerredWaitable<RecursiveLock>::execute([[maybe_unused]] std::shared_ptr<void> & data) {
    if(callback)
        callback();
}

/* ========================================================== Client API ========================================================== */

template<typename RecursiveLock>
void TriggerredWaitable<RecursiveLock>::trigger() { 
    
    // Mark event occurrence
    triggerred.store(true);
    // Trigger associated guard condition
    [[maybe_unused]] auto ret = rcl_trigger_guard_condition(&gc); 

    /**
     * @note Conducted empirical tests showned no impact on the code execution of the
     *    @ref rcl_trigger_guard_condition(...) call. This is however included in the
     *    [1] example and has been kept.
     * 
     * @see [1] https://answers.ros.org/question/322815/ros2-how-to-create-custom-waitable/
     */

}

/* ================================================================================================================================ */

} // End namespace utilities
} // End namespace velmwheel

#endif
