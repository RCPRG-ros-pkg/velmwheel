/* ============================================================================================================================ *//**
 * @file       service_callbacks.cpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 28th April 2022 9:24:14 pm
 * @modified   Monday, 18th July 2022 8:27:57 pm
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

        constexpr auto wheel_to_str = [](Wheels wheel) {
            switch(wheel) {
                case Wheels::RearLeft:   return "RearLeft";
                case Wheels::RearRight:  return "RearRight";
                case Wheels::FrontLeft:  return "FrontLeft";
                case Wheels::FrontRight: return "FrontRight";
                default:
                    return "<Unknown>";
            }   
        };
        
        RCLCPP_DEBUG_STREAM(node->get_logger(), "Reading cyclical data of [" << wheel_to_str(wheel) << "] wheel");

        // Get referene to the driver
        auto &driver = drivers[wheel];
        // Get referene to data
        auto &data = measurements.data[wheel];

        /* ----------------------------- Update measurements ------------------------------ */
        
        // Increment 'update' counter
        ++measurements.update_counter;

        // Update measurements of the servodriver
        data.position = (WheelsPolarities[wheel] * driver.get_position());
        data.velocity = (WheelsPolarities[wheel] * driver.get_velocity());

        bool valid_state_received = true;

        // Try to read the state of the servodriver
        try { data.state = driver.get_state(); }
        // If failed, print log
        catch(std::exception &ex) {
            RCLCPP_ERROR_STREAM(node->get_logger(), "Failed to read state of the [" << wheel_to_str(wheel) << "] wheel (" << ex.what() << ")");
            valid_state_received = false;
        }
        
        /* ------------------------------ Set servo's state ------------------------------- */

        if(valid_state_received){

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
                        case Driver::State::Fault:                                                                      break;
                        default: /* Should not happen */
                            invalid_state_error.arm(
                                "[velmwheel::BaseDriver::make_driver_callback] Servodriver found in invalid state"
                            );
                    }

                    // Check whether fault-reset request has been received
                    if(fault_reset_request.test(wheel)) {
                        
                        // Reset fault if wheel is faulty
                        if(data.state == Driver::State::Fault)
                            driver.set_command(Driver::Command::ResetFault);
                        // Reset reques flag of the current driver
                        fault_reset_request.reset(wheel);

                        // If all other drivers acknowledged their fault, switch state of the driver
                        if(fault_reset_request.none()) { 
                            
                            RCLCPP_INFO_STREAM(node->get_logger(), "Recovering from fault state...");

                            // Keep timestamp of recovery start
                            recovery_start = node->get_clock()->now();
                            // Set auxiliary bitset to trace what drivers leaved fault state
                            fault_recovery_status.set();
                            // Switch state
                            this->state = State::Recovering;
                        }

                    }

                    break;

                // Driver in 'Recovery' state
                case State::Recovering:

                    // If driver left fault state, mark it's flag
                    if(data.state != Driver::State::Fault)
                        fault_recovery_status.reset(wheel);

                    // If all other drivers acknowledged their fault, switch state of the driver
                    if(fault_recovery_status.none()) {
                    
                        RCLCPP_INFO_STREAM(node->get_logger(), "Sucesfully recovered from fault");

                        // Change state
                        this->state = State::Inactive;
                    
                    // Otherwise, if timeout expired, back to fault
                    } else if(node->get_clock()->now() - recovery_start >= FaultRecoveryTimeout) {
                    
                        RCLCPP_ERROR_STREAM(node->get_logger(), "Failed to recover from fault (timeout)");

                        // Change state
                        this->state = State::Fault;
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

    /**
     * @note First few measurements incoming from real servodrivers seem to be (partially) invalid.
     *    For this reason skip first few measurements to properly initialize odometry
     */
    if(initial_measurements_skip_counter != 0) {
        --initial_measurements_skip_counter;
        return;
    }

    // Get current timestamp for messages
    auto now = node->get_clock()->now();

    /* ------------------------------ Publish joints states ------------------------------- */

    sensor_msgs::msg::JointState joint_states_msg;

    // Resize vectors holding states of joints to the target size
    joint_states_msg.name    .reserve(Wheels::Num);
    joint_states_msg.position.reserve(Wheels::Num);
    joint_states_msg.velocity.reserve(Wheels::Num);
    joint_states_msg.effort  .reserve(Wheels::Num);
    // Prepare header of the encoders' measurements message
    joint_states_msg.header.stamp    = now;
    joint_states_msg.header.frame_id = velmwheel::params::ROBOT_NAME;
    // Fill the body of the encoders' measurements for rear left wheel
    joint_states_msg.name    .push_back("motor_rl");
    joint_states_msg.position.push_back(measurements.data[Wheels::RearLeft].position);
    joint_states_msg.velocity.push_back(measurements.data[Wheels::RearLeft].velocity);
    joint_states_msg.effort  .push_back(0);
    // Fill the body of the encoders' measurements for rear right wheel
    joint_states_msg.name    .push_back("motor_rr");
    joint_states_msg.position.push_back(measurements.data[Wheels::RearRight].position);
    joint_states_msg.velocity.push_back(measurements.data[Wheels::RearRight].velocity);
    joint_states_msg.effort  .push_back(0);
    // Fill the body of the encoders' measurements for front left wheel
    joint_states_msg.name    .push_back("motor_fl");
    joint_states_msg.position.push_back(measurements.data[Wheels::FrontLeft].position);
    joint_states_msg.velocity.push_back(measurements.data[Wheels::FrontLeft].velocity);
    joint_states_msg.effort  .push_back(0);
    // Fill the body of the encoders' measurements for front right wheel
    joint_states_msg.name    .push_back("motor_fr");
    joint_states_msg.position.push_back(measurements.data[Wheels::FrontRight].position);
    joint_states_msg.velocity.push_back(measurements.data[Wheels::FrontRight].velocity);
    joint_states_msg.effort  .push_back(0);
    // Update encoders' measurements
    joint_states_pub->publish(joint_states_msg);

    /* ------------------------------ Publish raw measurements ---------------------------- */
    
    velmwheel_msgs::msg::EncodersStamped encoders_msg;

    // Prepare header of the encoders' measurements message
    encoders_msg.header.stamp    = now;
    encoders_msg.header.frame_id = velmwheel::params::ROBOT_NAME;
    // Fill message with measurements
    encoders_msg.encoders[Wheels::RearLeft  ].angle    = measurements.data[Wheels::RearLeft  ].position;
    encoders_msg.encoders[Wheels::RearLeft  ].velocity = measurements.data[Wheels::RearLeft  ].velocity;
    encoders_msg.encoders[Wheels::RearRight ].angle    = measurements.data[Wheels::RearRight ].position;
    encoders_msg.encoders[Wheels::RearRight ].velocity = measurements.data[Wheels::RearRight ].velocity;
    encoders_msg.encoders[Wheels::FrontLeft ].angle    = measurements.data[Wheels::FrontLeft ].position;
    encoders_msg.encoders[Wheels::FrontLeft ].velocity = measurements.data[Wheels::FrontLeft ].velocity;
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
    status_msg.status[Wheels::RearLeft  ] = translate_status(measurements.data[Wheels::RearLeft  ].state);
    status_msg.status[Wheels::RearRight ] = translate_status(measurements.data[Wheels::RearRight ].state);
    status_msg.status[Wheels::FrontLeft ] = translate_status(measurements.data[Wheels::FrontLeft ].state);
    status_msg.status[Wheels::FrontRight] = translate_status(measurements.data[Wheels::FrontRight].state);

    // Publish the message
    status_pub->publish(status_msg);
}

/* ================================================================================================================================ */

} // End namespace velmwheel
