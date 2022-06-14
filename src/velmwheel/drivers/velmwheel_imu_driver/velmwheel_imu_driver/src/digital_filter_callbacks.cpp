/* ============================================================================================================================ *//**
 * @file       digital_filter_callbacks.cpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Monday, 30th May 2022 7:20:04 pm
 * @modified   Tuesday, 31st May 2022 4:42:52 pm
 * @project    engineering-thesis
 * @brief      Definition of callback methods of the ImuDriver class related to configruation of sensor's digital filter
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

void ImuDriver::get_digital_filter_callback(
    [[maybe_unused]] const velmwheel_imu_driver_msgs::srv::GetDigitalFilter::Request::SharedPtr req,
    velmwheel_imu_driver_msgs::srv::GetDigitalFilter::Response::SharedPtr res
) {

    uint16_t filter_setting;

    // Try to read current filter setting
    try {
        filter_setting = driver->read_digital_filter();
    // On error, return failure message
    } catch (std::exception &ex) {
        res->success       = false;
        res->error_message = ex.what();
        return;
    }

    // On success, fill response message
    res->filter  = filter_setting;
    res->success = true;
}


void ImuDriver::set_digital_filter_callback(
    const velmwheel_imu_driver_msgs::srv::SetDigitalFilter::Request::SharedPtr req,
    velmwheel_imu_driver_msgs::srv::SetDigitalFilter::Response::SharedPtr res
) {

    // Try to write current filter setting
    try {
        driver->write_digital_filter(req->filter);
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