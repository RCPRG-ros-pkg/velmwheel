/* ============================================================================================================================ *//**
 * @file       imu_driver.cpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 28th April 2022 9:24:14 pm
 * @modified   Wednesday, 29th June 2022 4:55:00 pm
 * @project    engineering-thesis
 * @brief      Definition of configruation methods of the ImuDriver class
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

/* =========================================================== Includes =========================================================== */

// Standard includes
#include <limits>
// Private includes
#include "node_common/communication.hpp"
#include "velmwheel/imu_driver.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace velmwheel {

/* ======================================================= Using namespaces ======================================================= */

using namespace node_common::parameters;
using namespace node_common::communication;

/* ================================================== Private API implementation ================================================== */

std::vector<std::string> ImuDriver::initialize(rclcpp::Node &node) {
    
    // Keep handle to the ROS node
    this->node = &node;

    /* ----------------------------- Initialize parameters --------------------------- */

    auto eni_name = declare_parameter_and_get(*(this->node), PARAM_NAMESPACE, ENI_NAME_PARAM_DESCRIPTOR);

    /* ------------------------------- Verify parameters ---------------------------- */

    // Verify whether non-emptydevice name has been given
    if(eni_name->size() == 0)
        rclcpp::exceptions::InvalidParametersException("'eni_name' cannot be an empty string");

    /* -------------------------------- Parse parameters ---------------------------- */

    // Return device name
    return std::vector<std::string>{ *eni_name };

}

void ImuDriver::configure(std::vector<cifx::ethercat::Slave*> slaves) {

    // Check if valid slave interfaces has been given
    if(slaves.size() != 1)
        throw std::runtime_error{ "[velmwhee::imu_driver] Driver has been configrued with invalid list of slave devices" };

    // Construct driver's implementation
    driver.emplace(*slaves[0]);

    /* ----------------------------- Initialize parameters --------------------------- */

    auto init_filter_seettings = declare_parameter_and_get(*(this->node), PARAM_NAMESPACE, INIT_FILTER_SEETTINGS_PARAM_DESCRIPTOR);
    auto init_gyro_range       = declare_parameter_and_get(*(this->node), PARAM_NAMESPACE, INIT_GYRO_RANGE_PARAM_DESCRIPTOR);

    /* ------------------------------- Verify parameters ---------------------------- */

    // Verify whether valid initial gyro range configruation given
    if(*init_gyro_range != 75 && *init_gyro_range != 150 && *init_gyro_range != 300)
        rclcpp::exceptions::InvalidParametersException("'init_gyro_range' must be one of [75, 150, 300]");

    // Verify whether valid digital filter constant given
    if(*init_filter_seettings < std::numeric_limits<uint16_t>::min() || *init_filter_seettings > std::numeric_limits<uint16_t>::max())
        rclcpp::exceptions::InvalidParametersException("'init_filter_seettings' must be in range of 'uint16' type");

    /* ------------------------------- Configure device ------------------------------ */

    // Configure new-measurement handler
    driver->set_measurement_read_handler(std::bind(&ImuDriver::imu_callback, this));

    // Configure initial digital filter constant
    driver->write_digital_filter(static_cast<uint16_t>(*init_filter_seettings));

    /**
     * Confiugre initial gyro range config (impossible at the moment)
     * 
     * @note For details see comment notes to the @ref ethercat::devices::Imu class in the [imu_ethercat] package
     */
    // switch(*init_gyro_range) {
    //     case 75:  driver->write_gyro_range(Driver::GyroRange::Range75);  break;
    //     case 150: driver->write_gyro_range(Driver::GyroRange::Range150); break;
    //     case 300: driver->write_gyro_range(Driver::GyroRange::Range300); break;
    //     default: /* Should not happen */
    //         std::runtime_error{ "[velmwheel::ImuDriver::initialize] Invalid gyro range parsed [BUG]" };
    // }
    
    RCLCPP_INFO_STREAM(node->get_logger(), "Configured [Imu] driver");

    /* ----------------------------- Initialize publishers --------------------------- */
            
    *make_publisher_builder(imu_pub)
        .node(*(this->node))
        .name(IMU_PUB_TOPIC_NAME)
        .qos(TOPIC_QUEUE_SIZE);
    
    /* ------------------------------ Initialize services ---------------------------- */

    // -- Measurement bias configruation services --- //

    *make_service_builder(get_acceleration_bias_offsets_srv)
        .node(*(this->node))
        .name(GET_ACCELERATION_BIAS_OFFSETS_SRV_TOPIC_NAME)
        .callback(*this, &ImuDriver::get_acceleration_bias_offsets_callback);

    *make_service_builder(set_acceleration_bias_offsets_srv)
        .node(*(this->node))
        .name(SET_ACCELERATION_BIAS_OFFSETS_SRV_TOPIC_NAME)
        .callback(*this, &ImuDriver::set_acceleration_bias_offsets_callback);

    *make_service_builder(get_gyro_bias_offsets_srv)
        .node(*(this->node))
        .name(GET_GYRO_BIAS_OFFSETS_SRV_TOPIC_NAME)
        .callback(*this, &ImuDriver::get_gyro_bias_offsets_callback);

    *make_service_builder(set_gyro_bias_offsets_srv)
        .node(*(this->node))
        .name(SET_GYRO_BIAS_OFFSETS_SRV_TOPIC_NAME)
        .callback(*this, &ImuDriver::set_gyro_bias_offsets_callback);

    *make_service_builder(get_bias_offsets_srv)
        .node(*(this->node))
        .name(GET_BIAS_OFFSETS_SRV_TOPIC_NAME)
        .callback(*this, &ImuDriver::get_bias_offsets_callback);

    *make_service_builder(set_bias_offsets_srv)
        .node(*(this->node))
        .name(SET_BIAS_OFFSETS_SRV_TOPIC_NAME)
        .callback(*this, &ImuDriver::set_bias_offsets_callback);

    // -- Digital filter configruation services --- //

    *make_service_builder(get_digital_filter_srv)
        .node(*(this->node))
        .name(GET_DIGITAL_FILTER_SRV_TOPIC_NAME)
        .callback(*this, &ImuDriver::get_digital_filter_callback);

    *make_service_builder(set_digital_filter_srv)
        .node(*(this->node))
        .name(SET_DIGITAL_FILTER_SRV_TOPIC_NAME)
        .callback(*this, &ImuDriver::set_digital_filter_callback);


    // -- Gyro range configruation services --- //

    *make_service_builder(get_gyro_range_srv)
        .node(*(this->node))
        .name(GET_GYRO_RANGE_SRV_TOPIC_NAME)
        .callback(*this, &ImuDriver::get_gyro_range_callback);

    *make_service_builder(set_gyro_range_srv)
        .node(*(this->node))
        .name(SET_GYRO_RANGE_SRV_TOPIC_NAME)
        .callback(*this, &ImuDriver::set_gyro_range_callback);

    // -- General configruation services --- //

    *make_service_builder(calibrate_gyro_bias_srv)
        .node(*(this->node))
        .name(CALIBRATE_GYRO_BIAS_SRV_TOPIC_NAME)
        .callback(*this, &ImuDriver::calibrate_gyro_bias_callback);

    *make_service_builder(calibrate_precise_gyro_bias_srv)
        .node(*(this->node))
        .name(CALIBRATE_PRECISE_GYRO_BIAS_SRV_TOPIC_NAME)
        .callback(*this, &ImuDriver::calibrate_precise_gyro_bias_callback);

    *make_service_builder(get_gyro_temperatures_srv)
        .node(*(this->node))
        .name(GET_GYRO_TEMPERATURES_SRV_TOPIC_NAME)
        .callback(*this, &ImuDriver::get_gyro_temperatures_callback);

    *make_service_builder(get_product_info_srv)
        .node(*(this->node))
        .name(GET_PRODUCT_INFO_SRV_TOPIC_NAME)
        .callback(*this, &ImuDriver::get_product_info_callback);

    *make_service_builder(restore_factory_calibration_srv)
        .node(*(this->node))
        .name(RESTORE_FACTORY_CALIBRATION_SRV_TOPIC_NAME)
        .callback(*this, &ImuDriver::restore_factory_calibration_callback);

    RCLCPP_INFO_STREAM(node->get_logger(), "Registered ROS interfaces for [Imu] driver");
}

/* ================================================================================================================================ */

} // End namespace velmwheel

/* ======================================================== Plugin includes ======================================================= */

#include <pluginlib/class_list_macros.hpp>

/* ======================================================== Plugins exports ======================================================= */

PLUGINLIB_EXPORT_CLASS(velmwheel::ImuDriver, velmwheel::EthercatSlaveDriver)

/* ================================================================================================================================ */
