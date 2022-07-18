/* ============================================================================================================================ *//**
 * @file       scan_data_parser.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Monday, 4th July 2022 3:06:48 pm
 * @modified   Thursday, 7th July 2022 5:30:37 pm
 * @project    engineering-thesis
 * @brief      Implementation of the inline methods of the ScanDataParser class
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __SICK_LMS1XX_SCAN_DATA_PARSER_SCAN_DATA_PARSER_H__
#define __SICK_LMS1XX_SCAN_DATA_PARSER_SCAN_DATA_PARSER_H__

/* =========================================================== Includes =========================================================== */

// Private includes
#include "sick/lms1xx/scan_data_parser.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace sick::lms1xx {

/* ====================================================== Auxiliary constants ===================================================== */

namespace scan_data_parser {

    /// Context string for log messages
    static constexpr auto context_base = "sick::lms1xx::ScanDataParser";

}

/* ======================================================== Auxiliary types ======================================================= */

bool &details::ScanDataFlagSet::operator[](LMS1xx::ScanData::ChannelContentType type) {
    return base_type::operator[](static_cast<std::size_t>(type));
}

/* ========================================================= Public ctors ========================================================= */

ScanDataParser::ScanDataParser(rclcpp::Logger *logger) :
    logger{ logger }
{ }

/* ================================================================================================================================ */

} // End namespace sick::lms1xx

/* ================================================================================================================================ */

#endif
