/* ============================================================================================================================ *//**
 * @file       configruation.cpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Tuesday, 5th July 2022 3:06:46 am
 * @modified   Thursday, 7th July 2022 7:42:45 pm
 * @project    engineering-thesis
 * @brief      ROS2-based class implementing LIDAR sensor driver node for the Velmwheel's driveline
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

/* =========================================================== Includes =========================================================== */

// Private includes
#include "velmwheel/laser_driver.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace velmwheel {

/* ======================================================== Private methods ======================================================= */

namespace details {

    static auto wait_with_timeout = [](
        rclcpp::Duration timeout,
        auto logger,
        auto clock,
        auto &&condition,
        std::string_view error_msg,
        std::string_view ongoing_msg
    ) {

        // Get connection start time
        auto connection_start = clock->now();

        // Wait for driver to connect
        while(not condition()) {

            // Sleep a while
            std::this_thread::sleep_for(std::chrono::seconds{ 1 });

            // Calculate timeout left
            auto timeout_left = (timeout - (clock->now() - connection_start));
            // If timeout reached, throw
            if(timeout_left.seconds() <= 0)
                throw std::runtime_error{ std::string{ error_msg } };

            RCLCPP_INFO_STREAM(logger, ongoing_msg << " (" << timeout_left.template to_chrono<std::chrono::seconds>().count() << "s)...");

        }
    };

}


void LaserDriver::connect_lidar() {
    
    RCLCPP_INFO_STREAM(this->get_logger(), "Connecting to the LIDAR...");

    // Connect to the driver
    driver->connect();

    // Wait for drver to connect
    details::wait_with_timeout(
        /* Timeout     */ std::chrono::seconds { 30 },
        /* Logger      */ this->get_logger(), 
        /* Clock       */ this->get_clock(), 
        /* Condition   */ [this](){ return driver->is_connected(); },
        /* Error msg   */ "[velmwheel::LaserDriver] Failed to connect to the LIDAR!", 
        /* Ongoing msg */ "Waiting for LIDAR to connect"
    );

    RCLCPP_INFO_STREAM(this->get_logger(), "Driver connected");
}


void LaserDriver::configure_scan(
    int scan_frequency,
    double angular_resolution
) {
    sick::LMS1xx::ScanConfig scan_config;

    // Make short aliases for the configuration values
    using AngularResolution = sick::LMS1xx::ScanConfig::AngularResolution;
    using ScanFrequency     = sick::LMS1xx::ScanConfig::ScanFrequency;

    // Prepare the config
    scan_config.scaning_frequency_hz = (scan_frequency     ==   25) ? ScanFrequency::Freq25Hz : ScanFrequency::Freq50Hz;
    scan_config.angular_step_size    = (angular_resolution == 0.25) ? AngularResolution::Res0_25deg : AngularResolution::Res0_50deg;

    RCLCPP_INFO_STREAM(this->get_logger(), "Configruing scan parameter...");

    // Set new configuration
    driver->set_scan_config(scan_config);

    // Calculate scanning period
    scanning_period = std::chrono::duration<float, std::ratio<1>>{ 1.0 / scan_frequency };

    RCLCPP_INFO_STREAM(this->get_logger(), "Reading range configuration...");

    // Read output range config
    scan_output_range_config = driver->get_scan_output_range();
}


void LaserDriver::configure_ntp(const std::string &hostname) {

    sick::LMS1xx::NtpConfig config;

    // If NTP is disable, return
    if(not use_ntp)
        return;

    RCLCPP_INFO_STREAM(this->get_logger(), "Configuring NTP server...");

    // Prepare basic configuration
    config.role                = sick::LMS1xx::NtpConfig::Role::Client;
    config.time_sync_interface = sick::LMS1xx::NtpConfig::TimeSyncInterface::Ethernet;
    config.time_zone           = std::chrono::hours{ 1 };
    config.update_time         = std::chrono::seconds{ 10 };
    // Parse IP address of the server
    sscanf(hostname.c_str(), "%d.%d.%d.%d", &config.server_ip[0], &config.server_ip[1], &config.server_ip[2], &config.server_ip[3]);

    // Setup the configuration
    driver->set_ntp_config(config);
}


void LaserDriver::configure_scan_data() {

    sick::LMS1xx::ScanDataConfig config;

    // Preapre configuration
    config.output_channel     = sick::LMS1xx::ScanDataConfig::OutputChannel::First;
    config.output_remission   = true;
    config.resolution         = sick::LMS1xx::ScanDataConfig::Resolution::Bit16;
    config.output_encoder     = false;
    config.output_position    = false;
    config.output_device_name = false;
    config.output_timestamp   = true;
    config.output_interval    = 1;

    RCLCPP_INFO_STREAM(this->get_logger(), "Configuring scan data format...");

    // Configure data format
    driver->set_scan_data_config(config);

}


void LaserDriver::configure_lidar(
    const std::string &hostname,
    int scan_frequency,
    double angular_resolution
) {

    // Connect to the LIDAR
    connect_lidar();
    // Login to the LIDAR
    driver->login();

    // Configure scan
    configure_scan(
        scan_frequency,
        angular_resolution
    );
    
    // Configure NTP
    configure_ntp(hostname);

    // Configrue scan data format
    configure_scan_data();
    
    // Start measuring
    driver->logout();
}


void LaserDriver::run_lidar() {

    // Start measuring
    driver->start_measurements();

    // Wait for device to start measuring
    details::wait_with_timeout(
        /* Timeout     */ std::chrono::seconds { 30 },
        /* Logger      */ this->get_logger(), 
        /* Clock       */ this->get_clock(), 
        /* Condition   */ [this](){ return (driver->get_status() == sick::LMS1xx::Status::ReadyForMeasurement); },
        /* Error msg   */ "[velmwheel::LaserDriver] Failed to bootup the LIDAR!", 
        /* Ongoing msg */ "Waiting for LIDAR to be ready for measuring"
    );

    RCLCPP_INFO_STREAM(this->get_logger(), "LIDAR booted up");
    // Enable continuous scan
    driver->enable_continous_scan(true);
}

/* ================================================================================================================================ */

} // End namespace velmwheel

/* ================================================================================================================================ */

