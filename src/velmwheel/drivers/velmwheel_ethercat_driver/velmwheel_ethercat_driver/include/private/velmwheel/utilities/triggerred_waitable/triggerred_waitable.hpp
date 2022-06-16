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
{ }


template<typename RecursiveLock>
size_t TriggerredWaitable<RecursiveLock>::get_number_of_ready_guard_conditions() { return 1; }


template<typename RecursiveLock>
void TriggerredWaitable<RecursiveLock>::add_to_wait_set(rcl_wait_set_t * wait_set) {
    gc.add_to_wait_set(wait_set);
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
    gc.trigger();

}

/* ================================================================================================================================ */

} // End namespace utilities
} // End namespace velmwheel

#endif
