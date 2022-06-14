/* ============================================================================================================================ *//**
 * @file       gyro_range_callbacks.cpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Monday, 30th May 2022 7:20:04 pm
 * @modified   Tuesday, 31st May 2022 4:31:04 pm
 * @project    engineering-thesis
 * @brief      Definition of callback methods of the ImuDriver class related to configruation of measurement range of gyro sensors
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

/* =========================================================== Includes =========================================================== */

// Private includes
#include "velmwheel/imu_driver.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace velmwheel {

/* =========================================================== Callbacks ========================================================== */

void ImuDriver::get_gyro_range_callback(
    [[maybe_unused]] const velmwheel_imu_driver_msgs::srv::GetGyroRange::Request::SharedPtr req,
    velmwheel_imu_driver_msgs::srv::GetGyroRange::Response::SharedPtr res
) {

    Driver::GyroRange gyro_range;

    // Try to read current range setting
    try {
        gyro_range = driver->read_gyro_range();
    // On error, return failure message
    } catch (std::exception &ex) {
        res->success       = false;
        res->error_message = ex.what();
        return;
    }

    // On success, Parse current value 
    switch(gyro_range) {
        case Driver::GyroRange::Range75:  res->range = velmwheel_imu_driver_msgs::srv::GetGyroRange::Response::RANGE_75;  break;
        case Driver::GyroRange::Range150: res->range = velmwheel_imu_driver_msgs::srv::GetGyroRange::Response::RANGE_150; break;
        case Driver::GyroRange::Range300: res->range = velmwheel_imu_driver_msgs::srv::GetGyroRange::Response::RANGE_300; break;
        default: /* Should not happen */
            throw std::runtime_error{ "[velmwheel::ImuDriver::get_gyro_range_callback] Invalid gyro range configuration parsed [BUG]" };
    }
    // Mark response as successful
    res->success = true;
}


void ImuDriver::set_gyro_range_callback(
    const velmwheel_imu_driver_msgs::srv::SetGyroRange::Request::SharedPtr req,
    velmwheel_imu_driver_msgs::srv::SetGyroRange::Response::SharedPtr res
) {

    Driver::GyroRange gyro_range;

    // Parse request
    switch(req->range) {
        
        case velmwheel_imu_driver_msgs::srv::SetGyroRange::Request::RANGE_75:  gyro_range = Driver::GyroRange::Range75;  break;
        case velmwheel_imu_driver_msgs::srv::SetGyroRange::Request::RANGE_150: gyro_range = Driver::GyroRange::Range150; break;
        case velmwheel_imu_driver_msgs::srv::SetGyroRange::Request::RANGE_300: gyro_range = Driver::GyroRange::Range300; break;

        // If invalid configuration given, return failure
        default: /* Should not happen */
            res->success       = false;
            res->error_message = "Invalid range identifier";
            return;
    }

    // Try to write current range setting
    try {
        driver->write_gyro_range(gyro_range);
    // On error, return failure message
    } catch (std::exception &ex) {
        res->success       = false;
        res->error_message = ex.what();
        return;
    }

    // On success, fill response message
    res->success = true;
}

/* ================================================================================================================================ */

} // End namespace velmwheel