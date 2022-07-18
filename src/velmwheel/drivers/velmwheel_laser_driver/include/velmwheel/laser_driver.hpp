/* ============================================================================================================================ *//**
 * @file       laser_driver.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Tuesday, 5th July 2022 2:46:01 am
 * @modified   Thursday, 7th July 2022 7:42:31 pm
 * @project    engineering-thesis
 * @brief      ROS2-based class implementing LIDAR sensor driver node for the Velmwheel's driveline
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __VELMWHEEL_LASER_DIVRER_H__
#define __VELMWHEEL_LASER_DIVRER_H__

/* =========================================================== Includes =========================================================== */

// System includes
#include <utility>
// ROS includes
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rclcpp_components/register_node_macro.hpp"
// Message includes
#include "sensor_msgs/msg/laser_scan.hpp"
// TF2 includes
#include "tf2_ros/static_transform_broadcaster.h"
// Private includes
#include "node_common.hpp"
#include "sick/lms1xx.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace velmwheel {

/* ============================================================= Node ============================================================= */

/**
 * @brief ROS2-based class implementing LIDAR sensor driver node for the Velmwheel's driveline
 */
class RCLCPP_PUBLIC LaserDriver : public rclcpp::Node {

public: /* -------------------------------------------------- Node's traits ------------------------------------------------------- */

    /// Name of the node
    static constexpr auto NODE_NAME = "lidar";

public: /* ------------------------------------------------ Node's parameters ----------------------------------------------------- */
    
    /// Description of the parameter determining IP of the host device
    static constexpr node_common::parameters::ParamDescriptor<std::string> HOSTNAME_PARAM_DESCRIPTOR {
        .name           = "hostname",
        .read_only      = true,
        .dynamic_typing = false,
        .default_value  = "192.168.0.1",
        .description    = "IP of the host device"
    };
    
    /// Description of the parameter determining IP of the LIDAR device
    static constexpr node_common::parameters::ParamDescriptor<std::string> DEVICENAME_PARAM_DESCRIPTOR {
        .name           = "devicename",
        .read_only      = true,
        .dynamic_typing = false,
        .default_value  = "192.168.0.2",
        .description    = "IP of the LIDAR sensor"
    };
    
    /// Description of the parameter determining topics that the plugin is publishing to
    static constexpr node_common::parameters::ParamDescriptor<std::string> PUBLISHING_MODE_PARAM_DESCRIPTOR {
        .name           = "publishing_mode",
        .read_only      = true,
        .dynamic_typing = false,
        .default_value  = "separate",
        .description    = "Publishing mode of the node indicating what topics it publishes to. Possible values are: \n"
                          "  'separate' - publishes measurements to ~/scan topic \n"
                          "  'common'   - publishes measurements to lidars/scan topic \n"
                          "  'both'     - publishes measurements to both topics"
    };

    /// Valid values of the node's parameter determining topics that the plugin is publishing to
    static constexpr std::array PUBLISHING_MODE_PARAM_VALID_VALUES { "separate", "common", "both" };

    /// Description of the parameter determining scan frequency of the LIDAR
    static constexpr node_common::parameters::ParamDescriptor<int> SCAN_FREQUENCY_PARAM_DESCRIPTOR {
        .name           = "scan_frequency",
        .read_only      = true,
        .dynamic_typing = false,
        .default_value  = 50,
        .description    = "Scan frequency of the LIDAR (one of {25, 50})"
    };

    /// Valid values of the node's parameter determining scan frequency of the LIDAR
    static constexpr std::array SCAN_FREQUENCY_PARAM_VALID_VALUES { 25, 50 };

    /// Description of the parameter determining angular resolution of the LIDAR
    static constexpr node_common::parameters::ParamDescriptor<double> ANGULAR_RESOLUTION_PARAM_DESCRIPTOR {
        .name           = "angular_resolution",
        .read_only      = true,
        .dynamic_typing = false,
        .default_value  = 0.25,
        .description    = "Angular resolution of the LIDAR (one of {0.25, 0,5})"
    };

    /// Valid values of the node's parameter determining angular resolution of the LIDAR
    static constexpr std::array ANGULAR_RESOLUTION_PARAM_VALID_VALUES { 0.25, 0.5 };

    /// Description of the parameter determining whether timing of scans will be filled witht he NTP usage
    static constexpr node_common::parameters::ParamDescriptor<bool> USE_NTP_PARAM_DESCRIPTOR {
        .name           = "use_ntp",
        .read_only      = true,
        .dynamic_typing = false,
        .default_value  = true,
        .description    = "If 'true', the driver will use NTP to controll data timestamps"
    };

    /// Description of the parameter determining reference frame of the published scans
    static constexpr node_common::parameters::ParamDescriptor<std::string> REFERENCE_FRAME_PARAM_DESCRIPTOR {
        .name           = "reference_frame",
        .read_only      = true,
        .dynamic_typing = false,
        .default_value  = "lidar_core",
        .description    = "Name of the reference frame of published scans"
    };

public: /* ------------------------------------------------ Topic's parameters ---------------------------------------------------- */
    
    /// Size of the topics' queue
    static constexpr std::size_t TOPIC_QUEUE_SIZE = 1000;

    /// Unqualified name of the topic that measurement are published onto
    static constexpr auto SCAN_TOPIC_NAME = "~/scan";
    /// Unqualified name of the topic that measurement are published onto (common for all LIDARs)
    static constexpr auto COMMON_SCAN_TOPIC_NAME = "lidars/scan";
    /// Scheme of the name of the base TF frame for the scan
    static constexpr auto SCAN_BASE_FRAME = "<node_name>_scan";

public: /* ----------------------------------------------- Public ctors & dtors --------------------------------------------------- */

    /**
     * @brief Construct a new LaserDriver node
     * 
     * @param options 
     *    configuration of the node
     */
    LaserDriver(const rclcpp::NodeOptions & options);

    /**
     * @brief Destructs LaserDriver node
     */
    ~LaserDriver();

private: /* ------------------------------------------------ Callback methods ----------------------------------------------------- */

    /**
     * @brief Callback method called at timer trigger
     */
    void measurement_callback();

private: /* ------------------------------------------------- Private methods ----------------------------------------------------- */

    /**
     * @brief Connects driver to the LIDAR device
     * 
     * @param devicename
     *    IP of the LIDAR sensor
     */
    void connect_lidar();

    /**
     * @brief Configures scan parameters of the LIDAR
     */
    void configure_scan(
        int scan_frequency,
        double angular_resolution
    );

    /**
     * @brief Configures NTP time source in the driver
     * 
     * @param hostname
     *    IP of the host device
     */
    void configure_ntp(const std::string &hostname);

    /**
     * @brief Configures scan data format
     */
    void configure_scan_data();

    /**
     * @brief Configures LIDAR driver
     * 
     * @param hostname
     *    IP of the host device
     * @param scan_frequency
     *    target scan frequency of the LIDAR
     * @param angular_resolution
     *    target angular resolution of the LIDAR
     */
    void configure_lidar(
        const std::string &hostname,
        int scan_frequency,
        double angular_resolution
    );

    /**
     * @brief Runs LIDAR scanning process
     */
    void run_lidar();
    
private: /* ------------------------------------------------- ROS interfaces ------------------------------------------------------ */

    /// Cyclic measurement-trigger timer
    rclcpp::TimerBase::SharedPtr timer;

	/// Publisher interface to broadcast LIDAR measurements on
	rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub{ nullptr };
	/// Publisher interface to broadcast LIDAR measurements on (common for all LIDARs)
	rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr common_pub{ nullptr };

    /// Name of the associated base TF frame
    std::string base_tf_frame;
    /// TF2 publishing object for broadcastign position of the LIDAR
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster;

private: /* -------------------------------------------------- Node's state ------------------------------------------------------- */

    /// If @c true driver will use NTP time
    bool use_ntp;

    /// Basic scan configuration
    sick::LMS1xx::ScanOutputRange scan_output_range_config;
    /// Preconfigured update period (in seconds)
    std::chrono::duration<float, std::ratio<1>> scanning_period;

    /// Scan data buffer
    sick::LMS1xx::ScanData scan_data;
    /// LIDAR driver class
    std::optional<sick::LMS1xx> driver;

};

/* ================================================================================================================================ */

} // End namespace velmwheel

#endif
