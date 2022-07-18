/* ============================================================================================================================ *//**
 * @file       scan_data_parser.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Monday, 4th July 2022 3:06:48 pm
 * @modified   Thursday, 7th July 2022 7:01:48 pm
 * @project    engineering-thesis
 * @brief      Auxiliary parser class automating scan data data parsing for LMS1xx driver class
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __SICK_LMS1XX_SCAN_DATA_PARSER_H__
#define __SICK_LMS1XX_SCAN_DATA_PARSER_H__

/* =========================================================== Includes =========================================================== */

// Standard includes
#include <cstdint>
// ROS includes
#include "rclcpp/rclcpp.hpp"
// Private includes
#include "sick/lms1xx.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace sick::lms1xx {
    
/* ======================================================== Auxiliary types ======================================================= */

namespace details {

    /**
     * @brief Auxiliary array type holding boolean flags indicating whether
     */
    struct ScanDataFlagSet : public std::array<bool, LMS1xx::ScanData::CHANNEL_TYPES_NUM> {

        using base_type = std::array<bool, LMS1xx::ScanData::CHANNEL_TYPES_NUM>;

        /// Forward constructors
        using base_type::array;
        
        /// Additional operator enabling set' indexing with content type enumerations
        inline bool &operator[](LMS1xx::ScanData::ChannelContentType type);
        /// Allow standard indexing usage
        using base_type::operator[];
    };

}

/* ============================================================= Class ============================================================ */

/**
 * @brief Auxiliary parser class automating scan data data parsing for LMS1xx driver class
 */
class ScanDataParser
{

public: /* -------------------------------------------------- Public ctors -------------------------------------------------------- */

    /**
     * @brief Construct a new ScanDataParser object with the given @p logger interface
     * 
     * @param logger 
     *    logger itnerface to be used
     * @param log_context 
     *    (optional) logger context string to be used
     */
    inline ScanDataParser(rclcpp::Logger *logger = nullptr);

public: /* ------------------------------------------------- Public methods ------------------------------------------------------- */

    /**
     * @brief Parses byte buffer @p data_buffer into the @ref LMS1xx::ScanData structure
     * 
     * @param data_buffer 
     *    buffer to be parsed
     * @param[out] parsed_data 
     *    parsed data buffer
     * @returns 
     *    parsed @ref ParsedData structure
     * 
     * @note For scan data structure see 'Telegram  Listing LiDAR sensors LMS1xx...' p. 71
     */
    void parse(char *data_buffer, LMS1xx::ScanData &parsed_data) const;

private: /* -------------------------------------- Private methods (header parsing) ----------------------------------------------- */

    /// Parsers header ('Command type' & 'Command' fields) of the scan data message
    void parse_header(char *data_buffer) const;

private: /* ------------------------------------ Private methods (device info parsing) -------------------------------------------- */
    
    /// Parses 'Version number' field of the scan data message
    uint16_t parse_version_number() const;
    
    /// Parses 'Device number' field of the scan data message
    uint16_t parse_device_number() const;
    
    /// Parses 'Serial number' field of the scan data message
    uint32_t parse_serial_number() const;
    
    /// Parses 'Device status' field of the scan data message
    LMS1xx::ScanData::DeviceStatus parse_device_status() const;

private: /* --------------------------------- Private methods (communication info parsing) ---------------------------------------- */

    /// Parses 'Telegram counter' field of the scan data message
    uint16_t parse_telegram_counter() const;
    
    /// Parses 'Scans counter' field of the scan data message
    uint16_t parse_scans_counter() const;
    
    /// Parses 'Time since start' field of the scan data message
    std::chrono::microseconds parse_time_since_start_us() const;
    
    /// Parses 'Time of transmission' field of the scan data message
    std::chrono::microseconds parse_time_of_transmission_us() const;

private: /* ------------------------------------- Private methods (digital I/O parsing) ------------------------------------------- */

    /// Parses 'Status of digital inputs' field of the scan data message
    std::bitset<2> parse_status_of_digital_inputs() const;
    
    /// Parses 'Status of digital outputs' field of the scan data message
    LMS1xx::ScanData::DigitalOutputsStatus parse_status_of_digital_outputs() const;

private: /* ------------------------------------- Private methods (scans info parsing) -------------------------------------------- */

    /// Parses 'Layer angle' field of the scan data message
    LMS1xx::ScanData::LayerAngle parse_layer_angle() const;
    
    /// Parses 'Scan frequency' field of the scan data message
    LMS1xx::ScanData::ScanFrequency parse_scan_frequency() const;
    
    /// Parses 'Measurements frequency' field of the scan data message
    uint32_t parse_measurements_frequency() const;

private: /* ------------------------------------ Private methods (encoders info parsing) ------------------------------------------ */
    
    /// Parses encoders info fields of the scan data message
    void parse_encoders_info(std::vector<LMS1xx::ScanData::EncoderInfo> &info) const;

private: /* ----------------------------------------- Private methods (data parsing) ---------------------------------------------- */

    /**
     * @brief Parses 16-bit data channels of the scan data message
     * 
     * @param[out] channel_data 
     *    output channels data structure
     * @param[out] parsed_content_flags
     *    set of boolean flags indicating what data types has already been parsed. Method will change state of corresponding
     *    flag(s) when aprsing data of the given type
     * 
     * @note 16-bit data parser is always called as the first, so @p parsed_content_flags should always have all 
     *    flags set to @c false .
     */
    void parse_channel_16_bit_data(
        LMS1xx::ScanData::DataSet &channel_data,
        details::ScanDataFlagSet &parsed_content_flags
    ) const;
    
    /**
     * @brief Parses 8-bit data channels of the scan data message
     * 
     * @param[out] channel_data 
     *    output channels data structure
     * @param[inout] parsed_content_flags
     *    set of boolean flags indicating what data types has already been parsed. Method will change state of corresponding
     *    flag(s) when aprsing data of the given type
     * 
     * @note 8-bit data parser is always called as the second, so if any of flags in @p parsed_content_flags is set to @c true ,
     *    the method will skip parsing this kind of data if it is also given with 8-bit representation (however it should not 
     *    happen, as the LIDAR itself will send data og the given type only with either 16-bit or 8-bit representation, never both)
     */
    void parse_channel_8_bit_data(
        LMS1xx::ScanData::DataSet &channel_data,
        details::ScanDataFlagSet &parsed_content_flags
    ) const;

    /// Parses data channels of the scan data message
    void parse_channel_data(LMS1xx::ScanData::DataSet &channel_data) const;

private: /* ------------------------------------ Private methods (auxiliary info parsing) ----------------------------------------- */

    /// Parses 'Name' field of the scan data message
    void parse_name() const;

    /// Parses 'Comment' field of the scan data message
    void parse_comment() const;

    /// Parses timestamp info fields of the scan data message
    std::optional<LMS1xx::ScanData::TimestampInfo> parse_timestamp_info() const;

private: /* ------------------------------------------------- Private data -------------------------------------------------------- */

    /// Optional logger interface
    rclcpp::Logger *logger;

    // Pointer to the buffer created by the strtok_r() call (used to make LMS1xx class thread-safe)
    mutable char *scan_data_strtok_save_ptr { nullptr };
    
};

/* ================================================================================================================================ */

} // End namespace sick::lms1xx

/* ==================================================== Implementation includes =================================================== */

#include "sick/lms1xx/scan_data_parser/scan_data_parser.hpp"

/* ================================================================================================================================ */

#endif
