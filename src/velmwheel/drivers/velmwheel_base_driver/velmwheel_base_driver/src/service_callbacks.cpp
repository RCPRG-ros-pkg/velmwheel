/* ============================================================================================================================ *//**
 * @file       service_callbacks.cpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 28th April 2022 9:24:14 pm
 * @modified   Tuesday, 14th June 2022 8:06:09 pm
 * @project    engineering-thesis
 * @brief      Definition of method of the BaseDriver class handling incoming service requests
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

/* =========================================================== Includes =========================================================== */

// Private includes
#include "velmwheel/base_driver.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace velmwheel {

/* ============================================================ Methods =========================================================== */

void BaseDriver::command_callback(
    const velmwheel_base_driver_msgs::srv::Command::Request::SharedPtr req,
    velmwheel_base_driver_msgs::srv::Command::Response::SharedPtr res
) {
    // If driver is not in a 'Fault' state, set 'Active' as a target state
    if(state != State::Fault) {

        // If enabling requested
        if(req->enable) {

            // Set new target state
            state = State::Active;
            // Return success
            res->success = true;
            
        // If disabling requested
        } else {

            // Set new target state
            state = State::Inactive;
            // Return success
            res->success = true;
        }


        return;

    // Otheriwse, return error
    } else {

        // Fill response
        res->success = false;
        res->error_message = "Driver is in the 'Fault' state";

        return;
    }
}


void BaseDriver::reset_failure_callback(
    [[maybe_unused]] const velmwheel_base_driver_msgs::srv::ResetFailure::Request::SharedPtr req,
    velmwheel_base_driver_msgs::srv::ResetFailure::Response::SharedPtr res
) {
    // If driver is in a a 'Fault' state, reset fault
    if(state == State::Fault) {

        // Reset all target speeds
        for(auto &driver : drivers)
            driver.set_velocity(0);

        // Set new reset request
        fault_reset_request.set();
        // Return success
        res->success = true;

        return;

    // Otherwise, return error
    } else {

        // Fill response
        res->success = false;
        res->error_message = "Driver is in not the 'Fault' state";

        return;
    }
}

/* ================================================================================================================================ */

} // End namespace velmwheel
