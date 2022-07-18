/* ============================================================================================================================ *//**
 * @file       imu.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 28th April 2022 9:24:14 pm
 * @modified   Friday, 1st July 2022 7:06:28 pm
 * @project    engineering-thesis
 * @brief      Definition of configruation methods of the Imu driver class
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __ETHERCAT_DEVICES_IMU_IMU_H__
#define __ETHERCAT_DEVICES_IMU_IMU_H__

/* =========================================================== Includes =========================================================== */

// Private includes
#include "ethercat/devices/imu.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace ethercat::devices {

/* ===================================================== Public ctors & dtors ===================================================== */

template<typename SlaveImplementationT>
Imu<SlaveImplementationT>::Imu(SlaveInterface &slave) :
    
    // Slave handler
    slave{ slave },
    // Device's configuration
    gyro_range { GyroRange::Range150 },
    // SDO interfaces
    gyro_temperature_x_sdo          { slave.template get_sdo<decltype(gyro_temperature_x_sdo         )>(imu::registers::GYRO_TEMPERATURE_X.index          ) },
    gyro_temperature_y_sdo          { slave.template get_sdo<decltype(gyro_temperature_y_sdo         )>(imu::registers::GYRO_TEMPERATURE_Y.index          ) },
    gyro_temperature_z_sdo          { slave.template get_sdo<decltype(gyro_temperature_z_sdo         )>(imu::registers::GYRO_TEMPERATURE_Z.index          ) },
    gyro_bias_offset_x_sdo          { slave.template get_sdo<decltype(gyro_bias_offset_x_sdo         )>(imu::registers::GYRO_BIAS_OFFSET_X.index          ) },
    gyro_bias_offset_y_sdo          { slave.template get_sdo<decltype(gyro_bias_offset_y_sdo         )>(imu::registers::GYRO_BIAS_OFFSET_Y.index          ) },
    gyro_bias_offset_z_sdo          { slave.template get_sdo<decltype(gyro_bias_offset_z_sdo         )>(imu::registers::GYRO_BIAS_OFFSET_Z.index          ) },
    accelerometer_bias_offset_x_sdo { slave.template get_sdo<decltype(accelerometer_bias_offset_x_sdo)>(imu::registers::ACCELEROMETER_BIAS_OFFSET_X.index ) },
    accelerometer_bias_offset_y_sdo { slave.template get_sdo<decltype(accelerometer_bias_offset_y_sdo)>(imu::registers::ACCELEROMETER_BIAS_OFFSET_Y.index ) },
    accelerometer_bias_offset_z_sdo { slave.template get_sdo<decltype(accelerometer_bias_offset_z_sdo)>(imu::registers::ACCELEROMETER_BIAS_OFFSET_Z.index ) },
    digital_filter_settings_sdo     { slave.template get_sdo<decltype(digital_filter_settings_sdo    )>(imu::registers::DIGITAL_FILTER_SETTINGS.index     ) },
    gyro_range_settings_sdo         { slave.template get_sdo<decltype(gyro_range_settings_sdo        )>(imu::registers::GYRO_RANGE_SETTINGS.index         ) },
    autonul_gyro_bias_sdo           { slave.template get_sdo<decltype(autonul_gyro_bias_sdo          )>(imu::registers::AUTONUL_GYRO_BIAS.index           ) },
    restore_factory_calibration_sdo { slave.template get_sdo<decltype(restore_factory_calibration_sdo)>(imu::registers::RESTORE_FACTORY_CALIBRATION.index ) },
    precision_gyro_bias_sdo         { slave.template get_sdo<decltype(precision_gyro_bias_sdo        )>(imu::registers::PRECISION_GYRO_BIAS.index         ) },
    product_id_sdo                  { slave.template get_sdo<decltype(product_id_sdo                 )>(imu::registers::PRODUCT_ID.index                  ) },
    serial_number_sdo               { slave.template get_sdo<decltype(serial_number_sdo              )>(imu::registers::SERIAL_NUMBER.index               ) },
    // PDO entry references
    acceleration_x_pdo   { slave.template get_pdo_entry<PdoDirection::Input>(imu::registers::ACCELERATION_X.name  ).template get_reference<decltype(acceleration_x_pdo)>()   },
    acceleration_y_pdo   { slave.template get_pdo_entry<PdoDirection::Input>(imu::registers::ACCELERATION_Y.name  ).template get_reference<decltype(acceleration_y_pdo)>()   },
    acceleration_z_pdo   { slave.template get_pdo_entry<PdoDirection::Input>(imu::registers::ACCELERATION_Z.name  ).template get_reference<decltype(acceleration_z_pdo)>()   },
    rotation_speed_x_pdo { slave.template get_pdo_entry<PdoDirection::Input>(imu::registers::ROTATION_SPEED_X.name).template get_reference<decltype(rotation_speed_x_pdo)>() },
    rotation_speed_y_pdo { slave.template get_pdo_entry<PdoDirection::Input>(imu::registers::ROTATION_SPEED_Y.name).template get_reference<decltype(rotation_speed_y_pdo)>() },
    rotation_speed_z_pdo { slave.template get_pdo_entry<PdoDirection::Input>(imu::registers::ROTATION_SPEED_Z.name).template get_reference<decltype(rotation_speed_z_pdo)>() }
    
{ 
    /**
     * @note Default configuration of gyro range and gitial filter SHOULD be set to:
     * 
     *      * "Digital Filter Settings" : N = 2^2       ( config: @c 2 )
     *      * "Gyro Range Settings"     : +/- 300 [°/s] ( config: @c 4 )
     * 
     *   These are default settings of the ADIS16362 IC ( @b SENS_AVG register ). However, due to some changes
     *   in the implementation of IMU's firmware, actual deffaults are different. According to the IMU driver 
     *   from the legacy [ec_drivers] package the default range set by the IMU is 150 [deg/s]. See warning note
     *   in the registers.hpp file for details
     * 
     * @see <a href="https://www.analog.com/media/en/technical-documentation/data-sheets/ADIS16362.pdf#page=10">ADIS16362 Datasheet (Table 8.)</a>  
     * @see <a href="https://www.analog.com/media/en/technical-documentation/data-sheets/ADIS16362.pdf#page=13">ADIS16362 Datasheet (Table 20.)</a>  
     */
    
    /**
     * Configure gyro range
     * 
     * @note This cannot be done at the moment ( @see warning comment in the registers.hpp )
     */
    // write_gyro_range(gyro_range);
    
    // Configure digital filter
    write_digital_filter(2);

}


template<typename SlaveImplementationT>
Imu<SlaveImplementationT>::~Imu() {

    // At teardown unregister inputs-update handler for the IMU device
    slave.unregister_event_handler(SlaveInterface::Event::InputsUpdate);

}

/* ================================================================================================================================ */

} // End namespace ethercat::devices

#endif
