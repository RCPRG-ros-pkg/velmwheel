/* ============================================================================================================================ *//**
 * @file       service_callbacks.cpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 28th April 2022 9:24:14 pm
 * @modified   Friday, 1st July 2022 6:09:11 pm
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

void BaseDriver::enable_callback(
    const velmwheel_base_driver_msgs::srv::Enable::Request::SharedPtr req,
    velmwheel_base_driver_msgs::srv::Enable::Response::SharedPtr res
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

void BaseDriver::get_state_callback(
    [[maybe_unused]] const velmwheel_base_driver_msgs::srv::GetState::Request::SharedPtr req,
    velmwheel_base_driver_msgs::srv::GetState::Response::SharedPtr res
) {
    // Parse current state
    switch(state) {
        case State::Inactive:   res->state = velmwheel_base_driver_msgs::srv::GetState::Response::INACTIVE;   break;
        case State::Active:     res->state = velmwheel_base_driver_msgs::srv::GetState::Response::ACTIVE;     break;
        case State::Fault:      res->state = velmwheel_base_driver_msgs::srv::GetState::Response::FAULT;      break;
        case State::Recovering: res->state = velmwheel_base_driver_msgs::srv::GetState::Response::RECOVERING; break;
        default:

            // If invalid state, prepare failure response
            res->success = false;
            // Compose error string
            res->error_message = std::string("Driver in invalid state (") + std::to_string(int(state)) + ")";

            return;
    }

    // On success, prepare sucesfull resopnse
    res->success = true;

    return;
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
