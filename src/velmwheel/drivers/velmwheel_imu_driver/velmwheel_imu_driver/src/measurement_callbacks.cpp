/* ============================================================================================================================ *//**
 * @file       measurement_callbacks.cpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Monday, 30th May 2022 7:20:04 pm
 * @modified   Tuesday, 28th June 2022 6:25:19 pm
 * @project    engineering-thesis
 * @brief      Definition of callback methods of the ImuDriver class related to measurements processing
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

void ImuDriver::imu_callback() {

    RCLCPP_DEBUG_STREAM(node->get_logger(), "Reading cyclical data of [Imu] sensor");

    // Read current measurements of the acceleration sensor
    auto acceleration_measurements = driver->get_acceleration_measurements();
    // Read current measurements of the gyro sensor
    auto gyro_measurements = driver->get_gyro_measurements();

    
    // Number of dimensions that the sensor measures values for
    constexpr std::size_t DIMENSIONS = 3;
    // Default covariance matrix for measurements
    std::array<double, DIMENSIONS * DIMENSIONS> default_covariance = 
    {
        0.001, 0.0,   0.0,
        0.0,   0.001, 0.0,
        0.0,   0.0,   0.001
    };

    sensor_msgs::msg::Imu msg { rosidl_runtime_cpp::MessageInitialization::ZERO };

    // Fill header of the message
    msg.header.frame_id = BASE_TF_FRAME;
    msg.header.stamp    = node->get_clock()->now();
    // Fill body of the message
    msg.linear_acceleration.x          = acceleration_measurements.x;
    msg.linear_acceleration.y          = acceleration_measurements.y;
    msg.linear_acceleration.z          = acceleration_measurements.z;
    msg.linear_acceleration_covariance = default_covariance;
    msg.angular_velocity.x             = gyro_measurements.x;
    msg.angular_velocity.y             = gyro_measurements.y;
    msg.angular_velocity.z             = gyro_measurements.z;
    msg.angular_velocity_covariance    = default_covariance;

    // Publish measurements
    imu_pub->publish(msg);
}

/* ================================================================================================================================ */

} // End namespace velmwheel