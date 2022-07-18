/* ============================================================================================================================ *//**
 * @file       imu.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 28th April 2022 9:24:14 pm
 * @modified   Friday, 1st July 2022 7:07:12 pm
 * @project    engineering-thesis
 * @brief      Definition of the EtherCAT driver implementation class for proprietary IMU sensor board by B. Kaczor
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __ETHERCAT_DEVICES_IMU_H__
#define __ETHERCAT_DEVICES_IMU_H__

/* =========================================================== Includes =========================================================== */

// Standard includes
#include <cmath>
// Private includes
#include "ethercat/slave.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace ethercat::devices {

/* ============================================================= Class ============================================================ */

/**
 * @brief Implementation of the EtherCAT driver for proprietary IMU sensor board by B. Kaczor
 * 
 * @tparam SlaveImplementationT 
 *    type of the object implementing hardware-dependent elements of the Slave interface
 */
template<typename SlaveImplementationT>
class Imu {

public: /* --------------------------------------------------- Public types ------------------------------------------------------ */

    // Type of the underlying driver
    using SlaveInterface = Slave<SlaveImplementationT>;

    /**
     * @brief Configration of the gyro sensors maximal range
     */
    enum class GyroRange : uint16_t {

        /// Maximal range configuration set to +/- 75°
        Range75 = 1,
        /// Maximal range configuration set to +/- 150°
        Range150 = 2,
        /// Maximal range configuration set to +/- 300°
        Range300 = 4
        
    };

    /**
     * @brief Pair-like structure holding product informations about the IMU sensor
     */
    struct ProductInfo {

        /// Product ID of the sensor
        uint16_t product_id;
        /// Serial number of the sensor
        uint16_t serial_num;
        
    };

    /**
     * @brief Triplet-like structure holding measurements/calibration values for all three axis
     */
    struct AxisValues {

        /// Measurement/caalibration for X axis
        double x;
        /// Measurement/caalibration for Y axis
        double y;
        /// Measurement/caalibration for Z axis
        double z;
        
    };

public: /* ------------------------------------------------- Public ctors & dtors ------------------------------------------------ */

    /**
     * @brief Constructs the driver
     * @param slave 
     *    handle to the device driver interface
     */
    inline Imu(SlaveInterface &slave);

    /**
     * @brief Destroy the Imu object cleaning up the driver interface
     */
    inline ~Imu();

    /// Disable copy-semantics
    Imu(const Imu &rimu) = delete;
    Imu &operator=(const Imu &rimu) = delete;
    /// Default move-semantics
    Imu(Imu &&rimu) = default;
    Imu &operator=(Imu &&rimu) = default;

public: /* ---------------------------------------- Public measurements-related methods ------------------------------------------ */

    /**
     * @brief Sets handler routine called when new IMU measurement is read from the bus
     * 
     * @tparam HandlerT 
     *    handler type
     * @param handler 
     *    handler to be set
     */
    template<typename HandlerT>
    inline void set_measurement_read_handler(HandlerT &&handler);

    /// Readscurrent measurements of the acceleration sensors from the device (values returned in [m/s^2])
    inline AxisValues get_acceleration_measurements();
    /// Readscurrent measurements of the gyro sensors from the device (values returned in [rad/s])
    inline AxisValues get_gyro_measurements();

public: /* ------------------------------------------------- Public I/O methods -------------------------------------------------- */

    /* -------------- Measurement bias configruation services -------------- */

    /// Reads bias offsets of acceleration sensors from the device (values returned in [m/s^2])
    AxisValues read_acceleration_bias_offsets();
    /// Writes bias offsets of acceleration sensors to the device (values expected in [m/s^2])
    void write_acceleration_bias_offsets(const AxisValues &offsets);
    /// Reads bias offsets of gyro sensors from the device (values returned in [rad/s])
    AxisValues read_gyro_bias_offsets();
    /// Writes bias offsets of gyro sensors to the device (values expected in [rad/s])
    void write_gyro_bias_offsets(const AxisValues &offsets);
    
    /* --------------- Digital filter configruation services --------------- */

    /// Reads configuration of the digiral filter from the device
    uint16_t read_digital_filter();
    /// Writes configuration of the digiral filter to the device
    void write_digital_filter(uint16_t filter);

    /* ----------------- Gyro range configruation services ----------------- */

    /// Reads configuration of the gyro sensors' range from the device ( @see warning comment in the registers.hpp )
    GyroRange read_gyro_range();
    /// Writes configuration of the gyro sensors' range to the device ( @see warning comment in the registers.hpp )
    void write_gyro_range(GyroRange range);

    /* ------------------ General configruation services ------------------- */

    /// Signals device to start calibration of bias offsets of gyro sensors
    void calibrate_gyro_bias();
    /// Signals device to start precise calibration of bias offsets of gyro sensors
    void calibrate_precise_gyro_bias();
    /// Reads current temperatures of gyro sensors (values returned in [°C])
    AxisValues get_gyro_temperatures();
    /// Reads product info from the device
    ProductInfo get_product_info();
    /// Signals device to restore configuration parameters to factory state
    void restore_factory_calibration();

private: /* ------------------------------------------------- Private types ------------------------------------------------------ */

    /// Alias for input SDO interface
    template<typename T>
    using Sdo = typename 
        SlaveInterface::template DefaultTranslatedSdo<SlaveInterface::SdoDirection::Bidirectional, T>;

    /// Alias for PDO direction enumeration
    using PdoDirection = typename SlaveInterface::PdoDirection;

    /// Alias for input PDO-entry reference object
    template<typename T>
    using InputPdoEntry = typename 
        SlaveInterface::template Pdo<PdoDirection::Input>::Entry::template DefaultTranslatedReference<T>;

    /// Alias for output PDO-entry reference object
    template<typename T>
    using OutputPdoEntry = typename 
        SlaveInterface::template Pdo<PdoDirection::Output>::Entry::template DefaultTranslatedReference<T>;

private: /* -------------------------------------------------- Private data ------------------------------------------------------ */

    /// Handle to the slave device interface
    SlaveInterface &slave;

    /// Current configuration of the gyro range
    GyroRange gyro_range;

private: /* ---------------------------------------------- Private data (SDO) ---------------------------------------------------- */

    /// SDO interface for accessing of the "Gyro Temperature X" CoE register
    Sdo<uint16_t> gyro_temperature_x_sdo;
    /// SDO interface for accessing of the "Gyro Temperature Y" CoE register
    Sdo<uint16_t> gyro_temperature_y_sdo;
    /// SDO interface for accessing of the "Gyro Temperature Z" CoE register
    Sdo<uint16_t> gyro_temperature_z_sdo;

    /// SDO interface for accessing of the "Gyro bias offset X" CoE register
    Sdo<uint16_t> gyro_bias_offset_x_sdo;
    /// SDO interface for accessing of the "Gyro bias offset Y" CoE register
    Sdo<uint16_t> gyro_bias_offset_y_sdo;
    /// SDO interface for accessing of the "Gyro bias offset Z" CoE register
    Sdo<uint16_t> gyro_bias_offset_z_sdo;

    /// SDO interface for accessing of the "Accelerometer bias offset X" CoE register
    Sdo<uint16_t> accelerometer_bias_offset_x_sdo;
    /// SDO interface for accessing of the "Accelerometer bias offset Y" CoE register
    Sdo<uint16_t> accelerometer_bias_offset_y_sdo;
    /// SDO interface for accessing of the "Accelerometer bias offset Z" CoE register
    Sdo<uint16_t> accelerometer_bias_offset_z_sdo;

    /// SDO interface for accessing of the "Digital Filter Settings" CoE register
    Sdo<uint16_t> digital_filter_settings_sdo;
    /// SDO interface for accessing of the "Gyro Range Settings" CoE register
    Sdo<GyroRange> gyro_range_settings_sdo;
    /// SDO interface for accessing of the "Autonull Gyro Bias" CoE register
    Sdo<bool> autonul_gyro_bias_sdo;
    /// SDO interface for accessing of the "Restore Factory Calibration" CoE register
    Sdo<bool> restore_factory_calibration_sdo;
    /// SDO interface for accessing of the "Precision Autonull Bias" CoE register
    Sdo<bool> precision_gyro_bias_sdo;

    /// SDO interface for accessing of the "Product ID" CoE register
    Sdo<uint16_t> product_id_sdo;
    /// SDO interface for accessing of the "Serial Number" CoE register
    Sdo<uint16_t> serial_number_sdo;

private: /* ---------------------------------------------- Private data (PDO) ---------------------------------------------------- */

    /// Reference to the "Acceleration X" entry in the Process Data Image buffer
    InputPdoEntry<int16_t> acceleration_x_pdo;
    /// Reference to the "Acceleration Y" entry in the Process Data Image buffer
    InputPdoEntry<int16_t> acceleration_y_pdo;
    /// Reference to the "Acceleration Z" entry in the Process Data Image buffer
    InputPdoEntry<int16_t> acceleration_z_pdo;

    /// Reference to the "Rotation Speed X" entry in the Process Data Image buffer
    InputPdoEntry<int16_t> rotation_speed_x_pdo;
    /// Reference to the "Rotation Speed Y" entry in the Process Data Image buffer
    InputPdoEntry<int16_t> rotation_speed_y_pdo;
    /// Reference to the "Rotation Speed Z" entry in the Process Data Image buffer
    InputPdoEntry<int16_t> rotation_speed_z_pdo;

};

/* ================================================================================================================================ */

} // End namespace ethercat::devices

/* ==================================================== Implementation includes =================================================== */

#include "ethercat/devices/imu/registers.hpp"
#include "ethercat/devices/imu/conversions.hpp"
#include "ethercat/devices/imu/imu.hpp"
#include "ethercat/devices/imu/bias.hpp"
#include "ethercat/devices/imu/filter.hpp"
#include "ethercat/devices/imu/general.hpp"
#include "ethercat/devices/imu/measurements.hpp"
#include "ethercat/devices/imu/range.hpp"

/* ================================================================================================================================ */

#endif
