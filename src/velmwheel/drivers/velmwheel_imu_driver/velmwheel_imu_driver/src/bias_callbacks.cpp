/* ============================================================================================================================ *//**
 * @file       bias_callbacks.cpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Monday, 30th May 2022 7:20:04 pm
 * @modified   Tuesday, 31st May 2022 4:30:25 pm
 * @project    engineering-thesis
 * @brief      Definition of callback methods of the ImuDriver class related to configruation of measurement bias
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

void ImuDriver::get_acceleration_bias_offsets_callback(
    [[maybe_unused]] const velmwheel_imu_driver_msgs::srv::GetAccelerationBiasOffsets::Request::SharedPtr req,
    velmwheel_imu_driver_msgs::srv::GetAccelerationBiasOffsets::Response::SharedPtr res
) {

    Driver::AxisValues offsets;

    // Try to read current bias offsets from the device
    try {
        offsets = driver->read_acceleration_bias_offsets();
    // On error, return failure message
    } catch (std::exception &ex) {
        res->success       = false;
        res->error_message = ex.what();
        return;
    }

    // On success, fill response message
    res->offsets.x = offsets.x;
    res->offsets.y = offsets.y;
    res->offsets.z = offsets.z;
    res->success   = true;
}


void ImuDriver::set_acceleration_bias_offsets_callback(
    const velmwheel_imu_driver_msgs::srv::SetAccelerationBiasOffsets::Request::SharedPtr req,
    velmwheel_imu_driver_msgs::srv::SetAccelerationBiasOffsets::Response::SharedPtr res
) {

    // Parse request
    Driver::AxisValues offsets {
        .x = req->offsets.x,
        .y = req->offsets.y,
        .z = req->offsets.z
    };

    // Try to write current bias offsets from the device
    try {
        driver->write_acceleration_bias_offsets(offsets);
    // On error, return failure message
    } catch (std::exception &ex) {
        res->success       = false;
        res->error_message = ex.what();
        return;
    }

    // On success, fill response message
    res->success = true;
}


void ImuDriver::get_gyro_bias_offsets_callback(
    [[maybe_unused]] const velmwheel_imu_driver_msgs::srv::GetGyroBiasOffsets::Request::SharedPtr req,
    velmwheel_imu_driver_msgs::srv::GetGyroBiasOffsets::Response::SharedPtr res
) {

    Driver::AxisValues offsets;

    // Try to read current bias offsets from the device
    try {
        offsets = driver->read_gyro_bias_offsets();
    // On error, return failure message
    } catch (std::exception &ex) {
        res->success       = false;
        res->error_message = ex.what();
        return;
    }

    // On success, fill response message
    res->offsets.x = offsets.x;
    res->offsets.y = offsets.y;
    res->offsets.z = offsets.z;
    res->success   = true;
}


void ImuDriver::set_gyro_bias_offsets_callback(
    const velmwheel_imu_driver_msgs::srv::SetGyroBiasOffsets::Request::SharedPtr req,
    velmwheel_imu_driver_msgs::srv::SetGyroBiasOffsets::Response::SharedPtr res
) {

    // Parse request
    Driver::AxisValues offsets {
        .x = req->offsets.x,
        .y = req->offsets.y,
        .z = req->offsets.z
    };

    // Try to write current bias offsets from the device
    try {
        driver->write_gyro_bias_offsets(offsets);
    // On error, return failure message
    } catch (std::exception &ex) {
        res->success       = false;
        res->error_message = ex.what();
        return;
    }

    // On success, fill response message
    res->success = true;
}


void ImuDriver::get_bias_offsets_callback(
    [[maybe_unused]] const velmwheel_imu_driver_msgs::srv::GetBiasOffsets::Request::SharedPtr req,
    velmwheel_imu_driver_msgs::srv::GetBiasOffsets::Response::SharedPtr res
) {

    Driver::AxisValues acceleration_offsets;
    Driver::AxisValues gyro_offsets;

    // Try to read current bias offsets from the device
    try {
        acceleration_offsets = driver->read_acceleration_bias_offsets();
        gyro_offsets         = driver->read_gyro_bias_offsets();
    // On error, return failure message
    } catch (std::exception &ex) {
        res->success       = false;
        res->error_message = ex.what();
        return;
    }

    // On success, fill response message
    res->acceleration.x = acceleration_offsets.x;
    res->acceleration.y = acceleration_offsets.y;
    res->acceleration.z = acceleration_offsets.z;
    res->gyro.x         = gyro_offsets.x;
    res->gyro.y         = gyro_offsets.y;
    res->gyro.z         = gyro_offsets.z;
    res->success   = true;
}


void ImuDriver::set_bias_offsets_callback(
    const velmwheel_imu_driver_msgs::srv::SetBiasOffsets::Request::SharedPtr req,
    velmwheel_imu_driver_msgs::srv::SetBiasOffsets::Response::SharedPtr res
) {

    // Parse request - acceleration biases
    Driver::AxisValues acceleration_offsets {
        .x = req->acceleration.x,
        .y = req->acceleration.y,
        .z = req->acceleration.z
    };

    // Parse request - gyri biases
    Driver::AxisValues gyro_offsets {
        .x = req->gyro.x,
        .y = req->gyro.y,
        .z = req->gyro.z
    };

    // Try to write current bias offsets from the device
    try {
        driver->write_acceleration_bias_offsets(acceleration_offsets);
        driver->write_gyro_bias_offsets(gyro_offsets);
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