/* ============================================================================================================================ *//**
 * @file       callbacks.cpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Tuesday, 5th July 2022 3:06:46 am
 * @modified   Thursday, 7th July 2022 7:56:16 pm
 * @project    engineering-thesis
 * @brief      ROS2-based class implementing LIDAR sensor driver node for the Velmwheel's driveline
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

/* =========================================================== Includes =========================================================== */

// Standard includes
#include <regex>
// Libraries includes
#include "tf2_eigen/tf2_eigen.hpp"
// Private includes
#include "velmwheel/laser_driver.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace velmwheel {

/* =========================================================== Callbacks ========================================================== */

void LaserDriver::measurement_callback() {

    RCLCPP_DEBUG_STREAM(this->get_logger(), "Reading LIDAR measurements...");

    /* ------------------- Get data from the sensor ------------------- */

    // Try to read scan data
    try { driver->get_scan_data(scan_data); }
    // On error, print warning
    catch(std::exception &ex) {
        RCLCPP_WARN_STREAM(this->get_logger(), "Failed to read LIDAR measurements (" << ex.what() << ")");
        return;
    }

    /* ------------------------ Format message ------------------------ */

    // Get current system time-point
    auto now = this->get_clock()->now();
    
    sensor_msgs::msg::LaserScan msg;
    
    // Auxiliary functor casting std::chrono::duration to floating point seconds
    auto to_seconds = [](auto value) {
        return std::chrono::duration_cast<std::chrono::duration<float, std::ratio<1>>>(value);
    };
    // Auxiliary functor casting std::chrono::duration to nanoseconds
    auto to_nanoseconds = [](auto value) {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(value);
    };

    // Find distance data in the scan data
    auto distances_data = scan_data.channel_data[sick::LMS1xx::ScanData::ChannelContentType::Dist1];
    // Find intensity data in the scan data
    auto intensities_data = scan_data.channel_data[sick::LMS1xx::ScanData::ChannelContentType::Rssi1];

    // If intensity data not received, throw error
    if (not distances_data.has_value()) {
        std::stringstream ss;
        ss << "[velmwheel::LaserDriver] Scan data does not contain distance data!";
        throw std::runtime_error{ ss.str() };
    }

    // If intensity data not received, throw error
    if (not intensities_data.has_value()) {
        std::stringstream ss;
        ss << "[velmwheel::LaserDriver] Scan data does not contain intensities data!";
        throw std::runtime_error{ ss.str() };
    }

    // If distance and intensity data are of unequal size, throw error
    if (distances_data->data.size() != intensities_data->data.size()) {
        std::stringstream ss;
        ss << "[velmwheel::LaserDriver] Distanc and intensity data differ in size! (" 
           << distances_data->data.size()   << " vs " 
           << intensities_data->data.size() << ")";
        throw std::runtime_error{ ss.str() };
    }

    // Calculate number of distance samples
    std::size_t distance_samples_num = distances_data->data.size();
    // Calculate number of intensity samples
    std::size_t intensity_samples_num = intensities_data->data.size();
    // Calculate number of samples 
    std::size_t samples_num = std::min(distance_samples_num, intensity_samples_num);
    // Calculate time between measurements in seconds (see docs)
    double time_increment_s = (1.0 / (scan_data.measurements_frequency * 100));

    // Fill the message's header
    msg.header.frame_id = base_tf_frame;
    // Fill the message's header (time)
    if(not use_ntp)
        msg.header.stamp = this->get_clock()->now();
    else if (not scan_data.timestamp_info.has_value()){
        std::stringstream ss;
        ss << "[velmwheel::LaserDriver] NTP has been enabled but scan data does not contain time info";
        throw std::runtime_error{ ss.str() };
    } else {
        msg.header.stamp.sec     = static_cast<int32_t> (scan_data.timestamp_info->seconds.count());
        msg.header.stamp.nanosec = static_cast<uint32_t>(to_nanoseconds(scan_data.timestamp_info->microseconds).count());
    }
    // Fill message's body
    msg.angle_min       = scan_output_range_config.start_angle_rad;
    msg.angle_max       = scan_output_range_config.stop_angle_rad;
    msg.angle_increment = scan_output_range_config.resolution_rad;
    msg.time_increment  = time_increment_s;
    msg.scan_time       = scanning_period.count();

    // Reset min/max range
    msg.range_min = std::numeric_limits<float>::max();
    msg.range_max = std::numeric_limits<float>::min();

    // Resize the output vectors
    msg.ranges.reserve(samples_num);
    msg.intensities.reserve(samples_num);
    // Fill message's output samples (convert distances from [mm] to [m])
    for(int i = 0; i < samples_num; i++) {

        // Keep range
        msg.ranges.push_back(distances_data  ->data[i]);
        // Keep intensity
        msg.intensities.push_back(intensities_data->data[i]);
        // Check if closer point found
        if(msg.range_min > distances_data->data[i])
            msg.range_min = distances_data->data[i];
        // Check if farer point found
        if(msg.range_max < distances_data->data[i])
            msg.range_max = distances_data->data[i];

    }
    
    /* ----------------------- Publish message ------------------------ */
        
    // Publish the message
    if(pub) 
        pub->publish(msg);
    // Publish the message (common topic)
    if(common_pub) 
        common_pub->publish(msg);
}

/* ================================================================================================================================ */

} // End namespace velmwheel

/* ================================================================================================================================ */

