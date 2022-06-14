/* ============================================================================================================================ *//**
 * @file       general_callbacks.cpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Monday, 30th May 2022 7:20:04 pm
 * @modified   Tuesday, 31st May 2022 4:30:55 pm
 * @project    engineering-thesis
 * @brief      Definition of callback methods of the ImuDriver class related to general configuration of the sensor
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

void ImuDriver::calibrate_gyro_bias_callback(
    [[maybe_unused]] const velmwheel_imu_driver_msgs::srv::CalibrateGyroBias::Request::SharedPtr req,
    velmwheel_imu_driver_msgs::srv::CalibrateGyroBias::Response::SharedPtr res
) {

    // Try to trigger calibration
    try {
        driver->calibrate_gyro_bias();
    // On error, return failure message
    } catch (std::exception &ex) {
        res->success       = false;
        res->error_message = ex.what();
        return;
    }

    // On success, fill response message
    res->success = true;
}


void ImuDriver::calibrate_precise_gyro_bias_callback(
    [[maybe_unused]] const velmwheel_imu_driver_msgs::srv::CalibratePreciseGyroBias::Request::SharedPtr req,
    velmwheel_imu_driver_msgs::srv::CalibratePreciseGyroBias::Response::SharedPtr res
) {

    // Try to trigger calibration
    try {
        driver->calibrate_precise_gyro_bias();
    // On error, return failure message
    } catch (std::exception &ex) {
        res->success       = false;
        res->error_message = ex.what();
        return;
    }

    // On success, fill response message
    res->success = true;
}


void ImuDriver::get_gyro_temperatures_callback(
    [[maybe_unused]] const velmwheel_imu_driver_msgs::srv::GetGyroTemperatures::Request::SharedPtr req,
    velmwheel_imu_driver_msgs::srv::GetGyroTemperatures::Response::SharedPtr res
) {

    Driver::AxisValues temperatures;

    // Try to read current temperatures
    try {
        temperatures = driver->get_gyro_temperatures();
    // On error, return failure message
    } catch (std::exception &ex) {
        res->success       = false;
        res->error_message = ex.what();
        return;
    }

    // On success, fill response message
    res->temperatures.x = temperatures.x;
    res->temperatures.y = temperatures.y;
    res->temperatures.z = temperatures.z;
    res->success        = true;
}


void ImuDriver::get_product_info_callback(
    [[maybe_unused]] const velmwheel_imu_driver_msgs::srv::GetProductInfo::Request::SharedPtr req,
    velmwheel_imu_driver_msgs::srv::GetProductInfo::Response::SharedPtr res
) {

    Driver::ProductInfo product_info;

    // Try to read current temperatures
    try {
        product_info = driver->get_product_info();
    // On error, return failure message
    } catch (std::exception &ex) {
        res->success       = false;
        res->error_message = ex.what();
        return;
    }

    // On success, fill response message
    res->product_id    = product_info.product_id;
    res->serial_number = product_info.serial_num;
    res->success       = true;
}


void ImuDriver::restore_factory_calibration_callback(
    [[maybe_unused]] const velmwheel_imu_driver_msgs::srv::RestoreFactoryCalibration::Request::SharedPtr req,
    velmwheel_imu_driver_msgs::srv::RestoreFactoryCalibration::Response::SharedPtr res
) {

    // Try to trigger factory settings restoration
    try {
        driver->restore_factory_calibration();
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