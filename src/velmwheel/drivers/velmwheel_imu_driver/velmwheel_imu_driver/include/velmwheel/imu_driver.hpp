/* ============================================================================================================================ *//**
 * @file       imu_driver.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 28th April 2022 9:24:14 pm
 * @modified   Monday, 18th July 2022 6:19:52 pm
 * @project    engineering-thesis
 * @brief      Definition of the driver plugin class for the IMU EtherCAT slave
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __VELMWHEEL_IMU_DRIVER_H__
#define __VELMWHEEL_IMU_DRIVER_H__

/* =========================================================== Includes =========================================================== */

// Standard includes
#include <cmath>
// ROS includes
#include "rclcpp/visibility_control.hpp"
// Interfaces includes
#include "sensor_msgs/msg/imu.hpp"
#include "velmwheel_imu_driver_msgs/srv/get_acceleration_bias_offsets.hpp"
#include "velmwheel_imu_driver_msgs/srv/set_acceleration_bias_offsets.hpp"
#include "velmwheel_imu_driver_msgs/srv/get_gyro_bias_offsets.hpp"
#include "velmwheel_imu_driver_msgs/srv/set_gyro_bias_offsets.hpp"
#include "velmwheel_imu_driver_msgs/srv/get_bias_offsets.hpp"
#include "velmwheel_imu_driver_msgs/srv/set_bias_offsets.hpp"
#include "velmwheel_imu_driver_msgs/srv/get_digital_filter.hpp"
#include "velmwheel_imu_driver_msgs/srv/set_digital_filter.hpp"
#include "velmwheel_imu_driver_msgs/srv/get_gyro_range.hpp"
#include "velmwheel_imu_driver_msgs/srv/set_gyro_range.hpp"
#include "velmwheel_imu_driver_msgs/srv/calibrate_gyro_bias.hpp"
#include "velmwheel_imu_driver_msgs/srv/calibrate_precise_gyro_bias.hpp"
#include "velmwheel_imu_driver_msgs/srv/get_gyro_temperatures.hpp"
#include "velmwheel_imu_driver_msgs/srv/get_product_info.hpp"
#include "velmwheel_imu_driver_msgs/srv/restore_factory_calibration.hpp"
// Private includes
#include "node_common/parameters.hpp"
#include "ethercat/devices/imu.hpp"
#include "velmwheel/ethercat_slave_driver.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace velmwheel {

/* ============================================================= Class ============================================================ */

/**
 * @brief Driver plugin class for the IMU EtherCAT slave
 */
class RCLCPP_PUBLIC ImuDriver : public EthercatSlaveDriver {

public: /* ------------------------------------------------ Node's parameters ----------------------------------------------------- */

    /// Namespace of the IMU driver's parameters
    static constexpr auto PARAM_NAMESPACE = "imu";

    /// Description of the parameter defining name of the sensor in the ENI file
    static constexpr node_common::parameters::ParamDescriptor<std::string> ENI_NAME_PARAM_DESCRIPTOR {
        .name                   = "eni_name",
        .read_only              = true,
        .dynamic_typing         = false,
        .default_value          = "Imu",
        .description            = "Name of the IMU slave used in the ENI configuration"
    };

    /// Description of the parameter defining initial configuration of digital filter setting
    static constexpr node_common::parameters::ParamDescriptor<int> INIT_FILTER_SEETTINGS_PARAM_DESCRIPTOR {
        .name                   = "init_filter_seettings",
        .read_only              = true,
        .dynamic_typing         = false,
        .default_value          = 2,
        .description            = "Initial configuration of the digital filter setting",
    };

    /// Description of the parameter defining initial configuration of gyro sensors range
    static constexpr node_common::parameters::ParamDescriptor<int> INIT_GYRO_RANGE_PARAM_DESCRIPTOR {
        .name                   = "init_gyro_range",
        .read_only              = true,
        .dynamic_typing         = false,
        .default_value          = 150,
        .description            = "Initial configuration of the gyro range as [+/- deg/s] limit. At the moment, this parameter is unused. "\
                                  "For details see comment notes to the [ethercat::devices::Imu] class in the [imu_ethercat] package",
        .additional_constraints = "One of [75, 150, 300]"
    };

public: /* ------------------------------------------------ Topic's parameters ---------------------------------------------------- */

    /// Size of the topics' queue
    static constexpr std::size_t TOPIC_QUEUE_SIZE = 50;
    /// Name of the TF reference frame
    static constexpr auto BASE_TF_FRAME = "imu_centre";
    
    /* ------ Output topic ------ */

    /// Unqualified name of the topic that measurement are published onto
    static constexpr auto IMU_PUB_TOPIC_NAME = "imu/out";

    /* ------ Measurement bias configruation services ------ */

    /// Name of the service topic used to read current bias offsets of acceleration sensors
    static constexpr auto GET_ACCELERATION_BIAS_OFFSETS_SRV_TOPIC_NAME = "imu/get_acceleration_bias_offsets";
    /// Name of the service topic used to set current bias offsets of acceleration sensors
    static constexpr auto SET_ACCELERATION_BIAS_OFFSETS_SRV_TOPIC_NAME = "imu/set_acceleration_bias_offsets";
    /// Name of the service topic used to read current bias offsets of gyro sensors
    static constexpr auto GET_GYRO_BIAS_OFFSETS_SRV_TOPIC_NAME = "imu/get_gyro_bias_offsets";
    /// Name of the service topic used to set current bias offsets of gyro sensors
    static constexpr auto SET_GYRO_BIAS_OFFSETS_SRV_TOPIC_NAME = "imu/set_gyro_bias_offsets";
    /// Name of the service topic used to read current bias offsets of both acceleration and gyro sensors
    static constexpr auto GET_BIAS_OFFSETS_SRV_TOPIC_NAME = "imu/get_bias_offsets";
    /// Name of the service topic used to set current bias offsets of both acceleration and gyro sensors
    static constexpr auto SET_BIAS_OFFSETS_SRV_TOPIC_NAME = "imu/set_bias_offsets";

    /* ------ Digital filter configruation services ------ */

    /// Name of the service topic used to read current configuration of digital filter of the sensor
    static constexpr auto GET_DIGITAL_FILTER_SRV_TOPIC_NAME = "imu/get_digital_filter";
    /// Name of the service topic used to set current configuration of digital filter of the sensor
    static constexpr auto SET_DIGITAL_FILTER_SRV_TOPIC_NAME = "imu/set_digital_filter";

    /* ------ Gyro range configruation services ------ */

    /// Name of the service topic used to read current configuration of range limit of gyro sensors
    static constexpr auto GET_GYRO_RANGE_SRV_TOPIC_NAME = "imu/get_gyro_range";
    /// Name of the service topic used to set current configuration of range limit of gyro sensors
    static constexpr auto SET_GYRO_RANGE_SRV_TOPIC_NAME = "imu/set_gyro_range";

    /* ------ General configruation services ------ */

    /// Name of the service topic used to trigger calibration of the gyro bias offsets
    static constexpr auto CALIBRATE_GYRO_BIAS_SRV_TOPIC_NAME = "imu/calibrate_gyro_bias";
    /// Name of the service topic used to trigger precise calibration of the gyro bias offsets
    static constexpr auto CALIBRATE_PRECISE_GYRO_BIAS_SRV_TOPIC_NAME = "imu/calibrate_precise_gyro_bias";
    /// Name of the service topic used to read current temperatures of gyro sensors
    static constexpr auto GET_GYRO_TEMPERATURES_SRV_TOPIC_NAME = "imu/get_gyro_temperatures";
    /// Name of the service topic used to read product info of the sensor
    static constexpr auto GET_PRODUCT_INFO_SRV_TOPIC_NAME = "imu/get_product_info";
    /// Name of the service topic used to restore factory calibration of sensor's parameters
    static constexpr auto RESTORE_FACTORY_CALIBRATION_SRV_TOPIC_NAME = "imu/restore_factory_calibration";

public: /* ------------------------------------------------- Public ctors & dtors ------------------------------------------------ */

    /**
     * @brief Constructs the driver
     */
    ImuDriver() = default;

    /**
     * @brief Destructs the driver undregistering all handlers from the Master
     */
    virtual ~ImuDriver() = default;

private: /* ------------------------------------------- Private implementation API ----------------------------------------------- */

    /**
     * @brief Initialization routine of the driver. At this step implementation can register 
     *    ROS-specific interfaces like topics, subscriptions and parameters declarations
     * 
     * @param node 
     *    handle to the ROS node interface
     * @returns 
     *    function should return list of names of the slave devices that this driver wants to manage
     */
    std::vector<std::string> initialize(rclcpp::Node &node) override;

    /**
     * @brief Function called by the loading process after @ref initialize . Provides driver
     *    with handle to the interface enabling slave device management
     * 
     * @param slave 
     *    list of handles to the slave devices interfaces
     */
    void configure(std::vector<cifx::ethercat::Slave*> slaves) override;

private: /* ---------------------------------------------- Private ROS callbacks ------------------------------------------------- */

    /* ------ Output topic ------ */

    /// Callback sending measurements read from the bus to the ROS system
    void imu_callback();

    /* ------ Measurement bias configruation services ------ */

    /// Callback managing service requests on topic used to read current bias offsets of acceleration sensors
    void get_acceleration_bias_offsets_callback(
        const velmwheel_imu_driver_msgs::srv::GetAccelerationBiasOffsets::Request::SharedPtr req,
        velmwheel_imu_driver_msgs::srv::GetAccelerationBiasOffsets::Response::SharedPtr res
    );
    
    /// Callback managing service requests on topic used to set current bias offsets of acceleration sensors
    void set_acceleration_bias_offsets_callback(
        const velmwheel_imu_driver_msgs::srv::SetAccelerationBiasOffsets::Request::SharedPtr req,
        velmwheel_imu_driver_msgs::srv::SetAccelerationBiasOffsets::Response::SharedPtr res
    );
    
    /// Callback managing service requests on topic used to read current bias offsets of gyro sensors
    void get_gyro_bias_offsets_callback(
        const velmwheel_imu_driver_msgs::srv::GetGyroBiasOffsets::Request::SharedPtr req,
        velmwheel_imu_driver_msgs::srv::GetGyroBiasOffsets::Response::SharedPtr res
    );
    
    /// Callback managing service requests on topic used to set current bias offsets of gyro sensors
    void set_gyro_bias_offsets_callback(
        const velmwheel_imu_driver_msgs::srv::SetGyroBiasOffsets::Request::SharedPtr req,
        velmwheel_imu_driver_msgs::srv::SetGyroBiasOffsets::Response::SharedPtr res
    );
    
    /// Callback managing service requests on topic used to read current bias offsets of both acceleration and gyro sensors
    void get_bias_offsets_callback(
        const velmwheel_imu_driver_msgs::srv::GetBiasOffsets::Request::SharedPtr req,
        velmwheel_imu_driver_msgs::srv::GetBiasOffsets::Response::SharedPtr res
    );
    
    /// Callback managing service requests on topic used to set current bias offsets of both acceleration and gyro sensors
    void set_bias_offsets_callback(
        const velmwheel_imu_driver_msgs::srv::SetBiasOffsets::Request::SharedPtr req,
        velmwheel_imu_driver_msgs::srv::SetBiasOffsets::Response::SharedPtr res
    );

    /* ------ Digital filter configruation services ------ */

    /// Callback managing service requests on topic used to read current configuration of digital filter of the sensor
    void get_digital_filter_callback(
        const velmwheel_imu_driver_msgs::srv::GetDigitalFilter::Request::SharedPtr req,
        velmwheel_imu_driver_msgs::srv::GetDigitalFilter::Response::SharedPtr res
    );
    
    /// Callback managing service requests on topic used to set current configuration of digital filter of the sensor
    void set_digital_filter_callback(
        const velmwheel_imu_driver_msgs::srv::SetDigitalFilter::Request::SharedPtr req,
        velmwheel_imu_driver_msgs::srv::SetDigitalFilter::Response::SharedPtr res
    );

    /* ------ Gyro range configruation services ------ */

    /// Callback managing service requests on topic used to read current configuration of range limit of gyro sensors
    void get_gyro_range_callback(
        const velmwheel_imu_driver_msgs::srv::GetGyroRange::Request::SharedPtr req,
        velmwheel_imu_driver_msgs::srv::GetGyroRange::Response::SharedPtr res
    );
    
    /// Callback managing service requests on topic used to set current configuration of range limit of gyro sensors
    void set_gyro_range_callback(
        const velmwheel_imu_driver_msgs::srv::SetGyroRange::Request::SharedPtr req,
        velmwheel_imu_driver_msgs::srv::SetGyroRange::Response::SharedPtr res
    );
    
    /* ------ General configruation services ------ */

    /// Callback managing service requests on topic used to trigger calibration of the gyro bias offsets
    void calibrate_gyro_bias_callback(
        const velmwheel_imu_driver_msgs::srv::CalibrateGyroBias::Request::SharedPtr req,
        velmwheel_imu_driver_msgs::srv::CalibrateGyroBias::Response::SharedPtr res
    );
    
    /// Callback managing service requests on topic used to trigger precise calibration of the gyro bias offsets
    void calibrate_precise_gyro_bias_callback(
        const velmwheel_imu_driver_msgs::srv::CalibratePreciseGyroBias::Request::SharedPtr req,
        velmwheel_imu_driver_msgs::srv::CalibratePreciseGyroBias::Response::SharedPtr res
    );
    
    /// Callback managing service requests on topic used to read current temperatures of gyro sensors
    void get_gyro_temperatures_callback(
        const velmwheel_imu_driver_msgs::srv::GetGyroTemperatures::Request::SharedPtr req,
        velmwheel_imu_driver_msgs::srv::GetGyroTemperatures::Response::SharedPtr res
    );
    
    /// Callback managing service requests on topic used to read product info of the sensor
    void get_product_info_callback(
        const velmwheel_imu_driver_msgs::srv::GetProductInfo::Request::SharedPtr req,
        velmwheel_imu_driver_msgs::srv::GetProductInfo::Response::SharedPtr res
    );
    
    /// Callback managing service requests on topic used to restore factory calibration of sensor's parameters
    void restore_factory_calibration_callback(
        const velmwheel_imu_driver_msgs::srv::RestoreFactoryCalibration::Request::SharedPtr req,
        velmwheel_imu_driver_msgs::srv::RestoreFactoryCalibration::Response::SharedPtr res
    );
    
private: /* --------------------------------------------- Private ROS interfaces ------------------------------------------------- */

    /* ------ Output topic ------ */

	/// Publisher interface used to broadcast IMU measurements
	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;

    /* ------ Measurement bias configruation services ------ */

    /// Service topic used to read current bias offsets of acceleration sensors
    rclcpp::Service<velmwheel_imu_driver_msgs::srv::GetAccelerationBiasOffsets>::SharedPtr get_acceleration_bias_offsets_srv;
    /// Service topic used to set current bias offsets of acceleration sensors
    rclcpp::Service<velmwheel_imu_driver_msgs::srv::SetAccelerationBiasOffsets>::SharedPtr set_acceleration_bias_offsets_srv;
    /// Service topic used to read current bias offsets of gyro sensors
    rclcpp::Service<velmwheel_imu_driver_msgs::srv::GetGyroBiasOffsets>::SharedPtr get_gyro_bias_offsets_srv;
    /// Service topic used to set current bias offsets of gyro sensors
    rclcpp::Service<velmwheel_imu_driver_msgs::srv::SetGyroBiasOffsets>::SharedPtr set_gyro_bias_offsets_srv;
    /// Service topic used to read current bias offsets of both acceleration and gyro sensors
    rclcpp::Service<velmwheel_imu_driver_msgs::srv::GetBiasOffsets>::SharedPtr get_bias_offsets_srv;
    /// Service topic used to set current bias offsets of both acceleration and gyro sensors
    rclcpp::Service<velmwheel_imu_driver_msgs::srv::SetBiasOffsets>::SharedPtr set_bias_offsets_srv;

    /* ------ Digital filter configruation services ------ */

    /// Service topic used to read current configuration of digital filter of the sensor
    rclcpp::Service<velmwheel_imu_driver_msgs::srv::GetDigitalFilter>::SharedPtr get_digital_filter_srv;
    /// Service topic used to set current configuration of digital filter of the sensor
    rclcpp::Service<velmwheel_imu_driver_msgs::srv::SetDigitalFilter>::SharedPtr set_digital_filter_srv;

    /* ------ Gyro range configruation services ------ */

    /// Service topic used to read current configuration of range limit of gyro sensors
    rclcpp::Service<velmwheel_imu_driver_msgs::srv::GetGyroRange>::SharedPtr get_gyro_range_srv;
    /// Service topic used to set current configuration of range limit of gyro sensors
    rclcpp::Service<velmwheel_imu_driver_msgs::srv::SetGyroRange>::SharedPtr set_gyro_range_srv;
    
    /* ------ General configruation services ------ */

    /// Service topic used to trigger calibration of the gyro bias offsets
    rclcpp::Service<velmwheel_imu_driver_msgs::srv::CalibrateGyroBias>::SharedPtr calibrate_gyro_bias_srv;
    /// Service topic used to trigger precise calibration of the gyro bias offsets
    rclcpp::Service<velmwheel_imu_driver_msgs::srv::CalibratePreciseGyroBias>::SharedPtr calibrate_precise_gyro_bias_srv;
    /// Service topic used to read current temperatures of gyro sensors
    rclcpp::Service<velmwheel_imu_driver_msgs::srv::GetGyroTemperatures>::SharedPtr get_gyro_temperatures_srv;
    /// Service topic used to read product info of the sensor
    rclcpp::Service<velmwheel_imu_driver_msgs::srv::GetProductInfo>::SharedPtr get_product_info_srv;
    /// Service topic used to restore factory calibration of sensor's parameters
    rclcpp::Service<velmwheel_imu_driver_msgs::srv::RestoreFactoryCalibration>::SharedPtr restore_factory_calibration_srv;

private: /* ------------------------------------------------- Private types ------------------------------------------------------ */

    /// Type of the driver class used by the node
    using Driver = ethercat::devices::Imu<cifx::ethercat::Slave>;

private: /* -------------------------------------------------- Private data ------------------------------------------------------ */

    /// Handle to the ROS node
    rclcpp::Node *node { nullptr };
    /// Driver implementation (optional to defer construction until configure() call)
    std::optional<Driver> driver;

};


/* ================================================================================================================================ */

} // End namespace velmwheel

#endif
