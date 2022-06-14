/* ============================================================================================================================ *//**
 * @file       service_callbacks.cpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 28th April 2022 9:24:14 pm
 * @modified   Tuesday, 14th June 2022 2:00:01 pm
 * @project    engineering-thesis
 * @brief      Definition of method of the BaseDriver class handling bus-synchronous data callbacks
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

/* =========================================================== Includes =========================================================== */

// Private includes
#include "velmwheel/base_driver.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace velmwheel {

/* ===================================================== Subscirbing callbacks ==================================================== */

void BaseDriver::setpoint_velocities_callback(const velmwheel_msgs::msg::Wheels &msg) {

    // Helper functor setting field of the current_controls_setpoint structure
    auto set_speed = [this, &msg](Wheels wheel) {
        if(!std::isnan(msg.values[wheel]))
            drivers[wheel].set_velocity(msg.values[wheel]);
    };

    // Set wheels' speed septoints
    set_speed(Wheels::RearLeft);
    set_speed(Wheels::RearRight);
    set_speed(Wheels::FrontLeft);
    set_speed(Wheels::FrontRight);

}

/* ===================================================== Publishing callbacks ===================================================== */

std::function<void(void)> BaseDriver::make_driver_callback(Wheels wheel) {
    return [this, wheel]() {

        // Get referene to the driver
        auto &driver = drivers[wheel];
        // Get referene to data
        auto &data = measurements.data[wheel];

        /* ----------------------------- Update measurements ------------------------------ */

        // Update data of the servodriver
        data.position = driver.get_position();
        data.velocity = driver.get_velocity();
        data.state    = driver.get_state();
        // Increment 'update' counter
        ++measurements.update_counter;
        
        /* ------------------------------ Set servo's state ------------------------------- */

        {
            /**
             * @brief Auxiliary RAII class providing means to report invalid state of the driver
             */
            class InvalidState {
            public:

                // Default cto
                InvalidState() = default;

                // RAII dtor throwing an exception on error
                ~InvalidState() noexcept(false) {
                    if(error_msg.has_value())
                        throw std::runtime_error{ *error_msg };
                }

                // Method arming the error
                void arm(std::string_view msg) { error_msg = msg; }

            private:

                // Message associated with the error
                std::optional<std::string> error_msg;

            } invalid_state_error;

            /*
             * Dispatch current state of the driver
             */
            switch(this->state) {

                // Driver in 'Inactive' state
                case State::Inactive:
        
                    /**
                     * In 'Inactive' state keep driver in 'ReadyToSwitchOn' state
                     */
                    switch(data.state) {
                        case Driver::State::NotReadyToSwitchOn:  driver.set_command(Driver::Command::DisableVoltage);   break;
                        case Driver::State::SwitchedOnDisabled:  driver.set_command(Driver::Command::Shutdown);         break;
                        case Driver::State::ReadyToSwitchOn:     driver.set_command(Driver::Command::Shutdown);         break;
                        case Driver::State::SwitchOn:            driver.set_command(Driver::Command::Shutdown);         break;
                        case Driver::State::OperationEnabled:    driver.set_command(Driver::Command::DisableOperation); break;
                        case Driver::State::QuickStopActive:                                                            break;
                        case Driver::State::FaultReactionAcitve:                                                        break;
                        case Driver::State::Fault:               this->state = State::Fault;                            break;
                        default: /* Should not happen */
                            invalid_state_error.arm(
                                "[velmwheel::BaseDriver::make_driver_callback] Servodriver found in invalid state"
                            );
                    }

                    break;

                // Driver in 'Active' state
                case State::Active:
                
                    /*
                     * In 'Inactive' state keep driver in 'ReadyToSwitchOn' state
                     */
                    switch(data.state) {
                        case Driver::State::NotReadyToSwitchOn:  driver.set_command(Driver::Command::DisableVoltage);  break;
                        case Driver::State::SwitchedOnDisabled:  driver.set_command(Driver::Command::Shutdown);        break;
                        case Driver::State::ReadyToSwitchOn:     driver.set_command(Driver::Command::SwitchOn);        break;
                        case Driver::State::SwitchOn:            driver.set_command(Driver::Command::EnableOperation); break;
                        case Driver::State::OperationEnabled:    driver.set_command(Driver::Command::EnableOperation); break;
                        case Driver::State::QuickStopActive:                                                           break;
                        case Driver::State::FaultReactionAcitve:                                                       break;
                        case Driver::State::Fault:               this->state = State::Fault;                           break;
                        default: /* Should not happen */
                            invalid_state_error.arm(
                                "[velmwheel::BaseDriver::make_driver_callback] Servodriver found in invalid state"
                            );
                    }

                    break;

                // Driver in 'Fault' state
                case State::Fault:

                    /*
                     * In 'Fault' state keep driver in 'ReadyToSwitchOn' state (if driver is
                     * actualy not the fault one)
                     */
                    switch(data.state) {
                        case Driver::State::NotReadyToSwitchOn:  driver.set_command(Driver::Command::DisableVoltage);   break;
                        case Driver::State::SwitchedOnDisabled:  driver.set_command(Driver::Command::Shutdown);         break;
                        case Driver::State::ReadyToSwitchOn:     driver.set_command(Driver::Command::Shutdown);         break;
                        case Driver::State::SwitchOn:            driver.set_command(Driver::Command::Shutdown);         break;
                        case Driver::State::OperationEnabled:    driver.set_command(Driver::Command::DisableOperation); break;
                        case Driver::State::QuickStopActive:                                                            break;
                        case Driver::State::FaultReactionAcitve:                                                        break;
                        case Driver::State::Fault:
                        
                            // Check whether fault-reset request has been received
                            if(fault_reset_request.test(wheel)) {
                                
                                // Reset fault
                                driver.set_command(Driver::Command::ResetFault);
                                // If all other drivers acknowledged their fault, switch state of the driver
                                if(fault_reset_request.none())
                                    this->state = State::Inactive;

                            }

                            break;

                        default: /* Should not happen */
                            invalid_state_error.arm(
                                "[velmwheel::BaseDriver::make_driver_callback] Servodriver found in invalid state"
                            );
                    }

                    break;

                // Should not happen
                default:
                    invalid_state_error.arm(
                        "[velmwheel::BaseDriver::make_driver_callback] Driver found in invalid state"
                    );

            }
        }

        /* ---------------------------- Dispatch ROS messaging ---------------------------- */

        // If all servos update their data since the last ROS publish, publis ne data
        if((measurements.update_counter = measurements.update_counter % Wheels::Num) == 0)
            broadcast_callback();
            
    };
}


void BaseDriver::broadcast_callback() {

    // Get current timestamp for messages
    auto now = node->get_clock()->now();

    /* ------------------------------ Publish joints states ------------------------------- */

    sensor_msgs::msg::JointState joint_states_msg;

    // Resize vectors holding states of joints to the target size
    joint_states_msg.name.reserve(Wheels::Num);
    joint_states_msg.position.reserve(Wheels::Num);
    joint_states_msg.velocity.reserve(Wheels::Num);
    joint_states_msg.effort.reserve(Wheels::Num);
    // Prepare header of the encoders' measurements message
    joint_states_msg.header.stamp    = now;
    joint_states_msg.header.frame_id = velmwheel::params::ROBOT_NAME;
    // Fill the body of the encoders' measurements for rear left wheel
    joint_states_msg.name.push_back("motor_rl");
    joint_states_msg.position.push_back(measurements.data[Wheels::RearLeft].position);
    joint_states_msg.velocity.push_back(measurements.data[Wheels::RearLeft].velocity);
    // Fill the body of the encoders' measurements for rear right wheel
    joint_states_msg.name.push_back("motor_rr");
    joint_states_msg.position.push_back(measurements.data[Wheels::RearRight].position);
    joint_states_msg.velocity.push_back(measurements.data[Wheels::RearRight].velocity);
    // Fill the body of the encoders' measurements for front left wheel
    joint_states_msg.name.push_back("motor_fl");
    joint_states_msg.position.push_back(measurements.data[Wheels::FrontLeft].position);
    joint_states_msg.velocity.push_back(measurements.data[Wheels::FrontLeft].velocity);
    // Fill the body of the encoders' measurements for front right wheel
    joint_states_msg.name.push_back("motor_fr");
    joint_states_msg.position.push_back(measurements.data[Wheels::FrontRight].position);
    joint_states_msg.velocity.push_back(measurements.data[Wheels::FrontRight].velocity);
    // Update encoders' measurements
    joint_states_pub->publish(joint_states_msg);

    /* ------------------------------ Publish raw measurements ---------------------------- */
    
    velmwheel_msgs::msg::EncodersStamped encoders_msg;

    // Prepare header of the encoders' measurements message
    encoders_msg.header.stamp    = now;
    encoders_msg.header.frame_id = velmwheel::params::ROBOT_NAME;
    // Fill message with measurements
    encoders_msg.encoders[Wheels::RearLeft].angle      = measurements.data[Wheels::RearLeft].position;
    encoders_msg.encoders[Wheels::RearLeft].velocity   = measurements.data[Wheels::RearLeft].velocity;
    encoders_msg.encoders[Wheels::RearRight].angle     = measurements.data[Wheels::RearRight].position;
    encoders_msg.encoders[Wheels::RearRight].velocity  = measurements.data[Wheels::RearRight].velocity;
    encoders_msg.encoders[Wheels::FrontLeft].angle     = measurements.data[Wheels::FrontLeft].position;
    encoders_msg.encoders[Wheels::FrontLeft].velocity  = measurements.data[Wheels::FrontLeft].velocity;
    encoders_msg.encoders[Wheels::FrontRight].angle    = measurements.data[Wheels::FrontRight].position;
    encoders_msg.encoders[Wheels::FrontRight].velocity = measurements.data[Wheels::FrontRight].velocity;
    
    // Publish the message
    encoders_pub->publish(encoders_msg);

    /* --------------------------------- Publish status info ------------------------------ */

    // Auxiliary method translating servo's status between app-domain and ROS message
    auto translate_status = [](Driver::State status) {
        switch(status) {
            case Driver::State::NotReadyToSwitchOn:  return velmwheel_base_driver_msgs::msg::WheelsStatus::NOT_READY_TO_SWITCH_ON;
            case Driver::State::SwitchedOnDisabled:  return velmwheel_base_driver_msgs::msg::WheelsStatus::SWITCHED_ON_DISABLED;
            case Driver::State::ReadyToSwitchOn:     return velmwheel_base_driver_msgs::msg::WheelsStatus::READY_TO_SWITCH_ON;
            case Driver::State::SwitchOn:            return velmwheel_base_driver_msgs::msg::WheelsStatus::SWITCH_ON;
            case Driver::State::OperationEnabled:    return velmwheel_base_driver_msgs::msg::WheelsStatus::OPERATION_ENABLED;
            case Driver::State::QuickStopActive:     return velmwheel_base_driver_msgs::msg::WheelsStatus::QUICK_STOP_ACTIVE;
            case Driver::State::FaultReactionAcitve: return velmwheel_base_driver_msgs::msg::WheelsStatus::FAULT_REACTION_ACITVE;
            case Driver::State::Fault:               return velmwheel_base_driver_msgs::msg::WheelsStatus::FAULT;
            default:
                return velmwheel_base_driver_msgs::msg::WheelsStatus::INVALID_STATE;
        }
    };

    velmwheel_base_driver_msgs::msg::WheelsStatus status_msg;

    // Prepare header of the encoders' measurements message
    status_msg.header.stamp    = now;
    status_msg.header.frame_id = velmwheel::params::ROBOT_NAME;
    // Parse servos' states
    status_msg.status[Wheels::RearLeft  ] = translate_status(drivers[Wheels::RearLeft  ].get_state());
    status_msg.status[Wheels::RearRight ] = translate_status(drivers[Wheels::RearRight ].get_state());
    status_msg.status[Wheels::FrontLeft ] = translate_status(drivers[Wheels::FrontLeft ].get_state());
    status_msg.status[Wheels::FrontRight] = translate_status(drivers[Wheels::FrontRight].get_state());

    // Publish the message
    status_pub->publish(status_msg);
}

/* ================================================================================================================================ */

} // End namespace velmwheel
