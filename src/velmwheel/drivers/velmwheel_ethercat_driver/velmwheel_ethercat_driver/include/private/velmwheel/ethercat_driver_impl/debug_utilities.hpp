/* ============================================================================================================================ *//**
 * @file       debug_utilities.cpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 28th April 2022 12:31:55 pm
 * @modified   Friday, 1st July 2022 6:37:33 pm
 * @project    engineering-thesis
 * @brief      Definitions of the debug utilities for the EtherCAT driver node of WUT Velmwheel robot
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __VELMWHEEL_ETHERCAT_DRIVER_IMPL_DEBUG_UTILITIES_H__
#define __VELMWHEEL_ETHERCAT_DRIVER_IMPL_DEBUG_UTILITIES_H__

/* =========================================================== Includes =========================================================== */

// System includes
#include <sys/mman.h>
#include <malloc.h>
// Private includes
#include "velmwheel/ethercat_driver_impl.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace velmwheel {

/* =================================================== Auxiliary debug function =================================================== */

namespace details::debug {

    /**
     * @brief Auxiliary debug-only function returning human-readable string representing current value
     *    of buffer related to particular slave devices on the EtherCAT bus in the Process Data Image
     * 
     * @param master 
     *   reference to the master interface
     * 
     * @note This function has ben left here only for potential future debug purposes
     */
    static inline std::string process_image_to_str(cifx::ethercat::Master &master) {

        using namespace ::ethercat::common::utilities;

        // Get buffer images from the Master
        auto input_buf  = master._get_input_buffer();
        auto output_buf = master._get_output_buffer();

        // Prepare auxiliary printing functors
        auto FMTI = [&input_buf](int id)  { return byte_to_str(input_buf[id]);  };
        auto FMTO = [&output_buf](int id) { return byte_to_str(output_buf[id]); };

        std::stringstream ss;

        // Print the image
        ss                                                                                                                                        << "\n";
        ss << "[velmwheel_ethercat_driver] Read input: "                                                                                          << "\n";
        ss                                                                                                                                        << "\n";
        ss << " - Imu.Transmit PDO Mapping.Acceleration X:      " << FMTI(71) << " " << FMTI(72)                                                  << "\n";
        ss << " - Imu.Transmit PDO Mapping.Acceleration Y:      " << FMTI(73) << " " << FMTI(74)                                                  << "\n";
        ss << " - Imu.Transmit PDO Mapping.Acceleration Z:      " << FMTI(75) << " " << FMTI(76)                                                  << "\n";
        ss << " - Imu.Transmit PDO Mapping.Rotation Speed X:    " << FMTI(77) << " " << FMTI(78)                                                  << "\n";
        ss << " - Imu.Transmit PDO Mapping.Rotation Speed Y:    " << FMTI(79) << " " << FMTI(80)                                                  << "\n";
        ss << " - Imu.Transmit PDO Mapping.Rotation Speed Z:    " << FMTI(81) << " " << FMTI(82)                                                  << "\n";
        ss                                                                                                                                        << "\n";
        ss << " - WheelRearLeft.Inputs.Position actual value:   " << FMTI(97)  << " " << FMTI(98)  << " " << FMTI(99)  << " " << FMTI(100) << " " << "\n";
        ss << " - WheelRearLeft.Inputs.Digital Inputs:          " << FMTI(101) << " " << FMTI(102) << " " << FMTI(103) << " " << FMTI(104) << " " << "\n";
        ss << " - WheelRearLeft.Inputs.Velocity actual value:   " << FMTI(105) << " " << FMTI(106) << " " << FMTI(107) << " " << FMTI(108) << " " << "\n";
        ss << " - WheelRearLeft.Inputs.Status word:             " << FMTI(109) << " " << FMTI(110)                                                << "\n";
        ss << " - WheelRearLeft.Outputs.Target:                 " << FMTO(97)  << " " << FMTO(98)  << " " << FMTO(99) << " " << FMTO(100)         << "\n";
        ss << " - WheelRearLeft.Outputs.Control:                " << FMTO(101) << " " << FMTO(102)                                                << "\n";
        ss                                                                                                                                        << "\n";
        ss << " - WheelRearRight.Inputs.Position actual value:  " << FMTI(111) << " " << FMTI(112) << " " << FMTI(113) << " " << FMTI(114) << " " << "\n";
        ss << " - WheelRearRight.Inputs.Digital Inputs:         " << FMTI(115) << " " << FMTI(116) << " " << FMTI(117) << " " << FMTI(118) << " " << "\n";
        ss << " - WheelRearRight.Inputs.Velocity actual value:  " << FMTI(119) << " " << FMTI(120) << " " << FMTI(121) << " " << FMTI(122) << " " << "\n";
        ss << " - WheelRearRight.Inputs.Status word:            " << FMTI(123) << " " << FMTI(124)                                                << "\n";
        ss << " - WheelRearRight.Outputs.Target:                " << FMTO(111) << " " << FMTO(112) << " " << FMTO(113) << " " << FMTO(114)        << "\n";
        ss << " - WheelRearRight.Outputs.Control:               " << FMTO(115) << " " << FMTO(116)                                                << "\n";
        ss                                                                                                                                        << "\n";
        ss << " - WheelFrontRight.Inputs.Position actual value: " << FMTI(125) << " " << FMTI(126) << " " << FMTI(127) << " " << FMTI(128) << " " << "\n";
        ss << " - WheelFrontRight.Inputs.Digital Inputs:        " << FMTI(129) << " " << FMTI(130) << " " << FMTI(131) << " " << FMTI(132) << " " << "\n";
        ss << " - WheelFrontRight.Inputs.Velocity actual value: " << FMTI(133) << " " << FMTI(134) << " " << FMTI(135) << " " << FMTI(136) << " " << "\n";
        ss << " - WheelFrontRight.Inputs.Status word:           " << FMTI(137) << " " << FMTI(138)                                                << "\n";
        ss << " - WheelFrontRight.Outputs.Target:               " << FMTO(125) << " " << FMTO(126) << " " << FMTO(127) << " " << FMTO(128)        << "\n";
        ss << " - WheelFrontRight.Outputs.Control:              " << FMTO(129) << " " << FMTO(130)                                                << "\n";
        ss                                                                                                                                        << "\n";
        ss << " - WheelFrontLeft.Inputs.Position actual value:  " << FMTI(139) << " " << FMTI(140) << " " << FMTI(141) << " " << FMTI(142) << " " << "\n";
        ss << " - WheelFrontLeft.Inputs.Digital Inputs:         " << FMTI(143) << " " << FMTI(144) << " " << FMTI(145) << " " << FMTI(146) << " " << "\n";
        ss << " - WheelFrontLeft.Inputs.Velocity actual value:  " << FMTI(147) << " " << FMTI(148) << " " << FMTI(149) << " " << FMTI(150) << " " << "\n";
        ss << " - WheelFrontLeft.Inputs.Status word:            " << FMTI(151) << " " << FMTI(152)                                                << "\n";
        ss << " - WheelFrontLeft.Outputs.Target:                " << FMTO(139) << " " << FMTO(140) << " " << FMTO(141) << " " << FMTO(142)        << "\n";
        ss << " - WheelFrontLeft.Outputs.Control:               " << FMTO(143) << " " << FMTO(144)                                                << "\n";
        ss                                                                                                                                        << "\n";
        ss << " - Image (IN):"                                                                                                                    << "\n";
        ss << buff_to_str(input_buf, 48, Color::Yellow)                                                                                           << "\n";
        ss << " - Image (OUT):"                                                                                                                   << "\n";
        ss << buff_to_str(output_buf, 48, Color::Red);

        return ss.str();
    }

    /**
     * @brief Auxiliary debug-only function printing human-readable string representing current value
     *    of buffer related to particular slave devices on the EtherCAT bus in the Process Data Image.
     *    Function shall be called cyclically. It will output log every @p period ms .
     * 
     * @param master 
     *   reference to the master interface
     * @param node 
     *   reference to the ROS node
     * @param period 
     *   reference to the ROS node
     * 
     * @note This function has ben left here only for potential future debug purposes
     */
    static inline void log_pdi_cyclically(
        cifx::ethercat::Master &master,
        rclcpp::Node &node,
        std::chrono::milliseconds period
    ) {

        // Timestamp of the last log
        static std::optional<decltype(node.get_clock()->now())> last;

        // Initialize timestamp on first log
        if(not last.has_value())
            last.emplace(node.get_clock()->now() - period);

        // If period elapsed, log
        if(*last + period <= node.get_clock()->now()) {
            
            // Log
            RCLCPP_INFO_STREAM(node.get_logger(), process_image_to_str(master) << std::endl);
            // Update log timestamp
            last = node.get_clock()->now();
        }
    }

}

/* ================================================================================================================================ */

} // End namespace velmwheel

#endif
