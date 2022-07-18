/* ============================================================================================================================ *//**
 * @file       scan_data_parser.cpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Monday, 4th July 2022 3:06:48 pm
 * @modified   Thursday, 7th July 2022 7:33:00 pm
 * @project    engineering-thesis
 * @brief      Implementation of the methods of the lms1xx
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

/* =========================================================== Includes =========================================================== */

// Standard includes
#include <cstring>
#include <cstdarg>
// Private includes
#include "sick/lms1xx.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace sick::lms1xx {

/* ======================================================= Auxiliary macros ======================================================= */

/// Logs debug message in a stream-like fashion, if logger is configrued
#define LOG_DEBUG(...)    \
    if(logger != nullptr) \
        RCLCPP_DEBUG_STREAM(*logger, "[" << scan_data_parser::context_base << "::" << context << "] " << __VA_ARGS__)

/// Logs warning message in a stream-like fashion, if logger is configrued
#define LOG_WARNING(...)    \
    if(logger != nullptr) \
        RCLCPP_WARN_STREAM(*logger, "[" << scan_data_parser::context_base << "::" << context << "] " << __VA_ARGS__)

/* =========================================================== Constants ========================================================== */

/// Log context
constexpr std::string_view context = "parse";

/* ====================================================== Auxiliary functions ===================================================== */

/// Casts @p t to its underlying type
template<typename Enum>
std::underlying_type_t<Enum> to_underlying(Enum e) {
    return static_cast<std::underlying_type_t<Enum>>(e);
}

/// Casts @p t to Enum type
template<typename Enum, typename T>
Enum to_enum(T t) {
    return static_cast<Enum>(t);
}

/// Converts @p angle from [1/10'000 deg] to [rad]
template<typename T>
double lidar_angle_to_rad(T angle) {
    return ((static_cast<double>(angle) * 2 * M_PI / 10'000.0) / 360.0);
}

/* ======================================================= Auxiliary methods ====================================================== */

namespace details {

    /**
     * @brief Initializes strtok_r() function with the given pointers parsing first token in the buffer
     * 
     * @param data_buffer 
     *    pointer to the buffer to be parsed
     * @param scan_data_strtok_save_ptr 
     *    pointer to the thread-safe tokenizer buffer
     * 
     * @returns 
     *    parsed token
     */
    static inline char *parse_first_token(char *data_buffer, char **scan_data_strtok_save_ptr) {
        return strtok_r(data_buffer, " ", scan_data_strtok_save_ptr);
    }

    /**
     * @brief Parses next token in the buffer initialized with @ref parse_first_token() call
     * 
     * @param scan_data_strtok_save_ptr 
     *    pointer to the thread-safe tokenizer buffer
     * 
     * @returns 
     *    parsed token
     */
    static inline char *parse_token(char **scan_data_strtok_save_ptr) {
        return strtok_r(nullptr, " ", scan_data_strtok_save_ptr);
    }

    /**
     * @brief Parses next token in the buffer initialized with @ref parse_first_token() call. Calls
     *    sscanf(...) on the token with the given args...
     * 
     * @param scan_data_strtok_save_ptr 
     *    pointer to the thread-safe tokenizer buffer
     * @param format 
     *    scanf format
     * @param ... 
     *    scanf arguments
     * 
     * @returns 
     *    parsed token
     */
    static inline char *parse_token_with_args(char **scan_data_strtok_save_ptr, const char *format, ...) {

        // Parse token
        auto token = parse_token(scan_data_strtok_save_ptr);

        std::va_list args;

        // Initialize va_list
        va_start(args, format);
        // Sscan the token
        vsscanf(token, format, args);
        // Deinitiliaze va_list
        va_end(args);

        return token;
    }

    /**
     * @brief Skipps next (n - 1) tokens in the buffer initialized with @ref skip_first_token() call
     *    and parses the next token
     * 
     * @param n 
     *    number of tokens to be parsed
     * @param scan_data_strtok_save_ptr 
     *    pointer to the thread-safe tokenizer buffer
     * 
     * @returns 
     *    parsed token
     */
    static inline char *parse_nth_token(
        std::size_t n,
        char **scan_data_strtok_save_ptr
    ) {
        // Skipp (n - 1) tokens
        for(std::size_t i = 1; i < n; ++i)
            parse_token(scan_data_strtok_save_ptr);

        // Parse the last token
        return parse_token(scan_data_strtok_save_ptr);
    }

    /**
     * @brief Skipps next (n - 1) tokens in the buffer initialized with @ref skip_first_token() call
     *    and parses the next token. Calls
     *    sscanf(...) on the token with the given args...
     * 
     * @param n 
     *    number of tokens to be parsed
     * @param scan_data_strtok_save_ptr 
     *    pointer to the thread-safe tokenizer buffer
     * @param format
     *    scanf format
     * @param ... 
     *    scanf arguments
     * 
     * @returns 
     *    parsed token
     */
    static inline char *parse_nth_token_with_args(
        std::size_t n,
        char **scan_data_strtok_save_ptr,
        const char *format, ...
    ) {
        // Skipp (n - 1) tokens
        for(std::size_t i = 1; i < n; ++i)
            parse_token(scan_data_strtok_save_ptr);

        // Parse the last token
        auto token = parse_token(scan_data_strtok_save_ptr);

        std::va_list args;

        // Initialize va_list
        va_start(args, format);
        // Sscan the token
        vsscanf(token, format, args);
        // Deinitiliaze va_list
        va_end(args);

        return token;
    }

}

/* =============================================== Private methods (header parsing) =============================================== */

void ScanDataParser::parse_header(char *data_buffer) const {

    // Initialize tokenizer skipping first data field ('Command type')
    details::parse_first_token(data_buffer, &scan_data_strtok_save_ptr);
    // Skip 'Command' field
    details::parse_token(&scan_data_strtok_save_ptr);

}

/* ============================================= Private methods (device info parsing) ============================================ */

uint16_t ScanDataParser::parse_version_number() const {

    uint16_t version_number;

    // Parse token
    details::parse_token_with_args(&scan_data_strtok_save_ptr, "%hX", &version_number);

    LOG_DEBUG(" - version number: 0x" << std::hex << version_number);

    return version_number;
}


uint16_t ScanDataParser::parse_device_number() const {

    uint16_t device_number;

    // Parse token
    details::parse_token_with_args(&scan_data_strtok_save_ptr, "%hX", &device_number);

    LOG_DEBUG(" - device number: 0x" << std::hex << device_number);

    return device_number;
}


uint32_t ScanDataParser::parse_serial_number() const {

    uint32_t serial_number;

    // Parse token
    details::parse_token_with_args(&scan_data_strtok_save_ptr, "%hX", &serial_number);

    LOG_DEBUG(" - serial number: 0x" << std::hex << serial_number);

    return serial_number;
}


LMS1xx::ScanData::DeviceStatus ScanDataParser::parse_device_status() const {

    uint8_t device_status;

    // Parse token
    details::parse_nth_token_with_args(2, &scan_data_strtok_save_ptr, "%hhX", &device_status);

    LOG_DEBUG(" - device status: 0x" << std::hex << int(device_status));

    // Convert status to enumeration
    switch(device_status) {
        case 0: return LMS1xx::ScanData::DeviceStatus::Ok;                           break;
        case 1: return LMS1xx::ScanData::DeviceStatus::Error;                        break;
        case 2: return LMS1xx::ScanData::DeviceStatus::PollutionWarning;             break;
        case 4: return LMS1xx::ScanData::DeviceStatus::PollutionErrorWithNoDevError; break;
        case 5: return LMS1xx::ScanData::DeviceStatus::PollutionErrorWithDevError;   break;
        default: /* Should not happen */
            LOG_WARNING("Invalid device status received (" << int(device_status) << ")");
    }

    return to_enum<LMS1xx::ScanData::DeviceStatus>(device_status);
}

/* ========================================= Private methods (communication info parsing) ========================================= */

uint16_t ScanDataParser::parse_telegram_counter() const {

    uint16_t telegram_counter;

    // Parse token
    details::parse_token_with_args(&scan_data_strtok_save_ptr, "%hX", &telegram_counter);

    LOG_DEBUG(" - telegram counter: " << std::dec << telegram_counter);

    return telegram_counter;
}


uint16_t ScanDataParser::parse_scans_counter() const {

    uint16_t scans_counter;

    // Parse token
    details::parse_token_with_args(&scan_data_strtok_save_ptr, "%hX", &scans_counter);

    LOG_DEBUG(" - scans counter: " << std::dec << scans_counter);

    return scans_counter;
}


std::chrono::microseconds ScanDataParser::parse_time_since_start_us() const {

    uint32_t time_since_start_us;

    // Parse token
    details::parse_token_with_args(&scan_data_strtok_save_ptr, "%X", &time_since_start_us);

    LOG_DEBUG(" - time since start: " << std::dec << time_since_start_us << " [us]");

    return std::chrono::microseconds{ time_since_start_us };
}


std::chrono::microseconds ScanDataParser::parse_time_of_transmission_us() const {

    uint32_t time_of_transmission_us;

    // Parse token
    details::parse_token_with_args(&scan_data_strtok_save_ptr, "%X", &time_of_transmission_us);

    LOG_DEBUG(" - time of transmission: " << std::dec << time_of_transmission_us << " [us]");

    return std::chrono::microseconds{ time_of_transmission_us };
}

/* ============================================= Private methods (digital I/O parsing) ============================================ */

std::bitset<2> ScanDataParser::parse_status_of_digital_inputs() const {

    uint8_t byte;

    // Parse token
    details::parse_token_with_args(&scan_data_strtok_save_ptr, "%hhX", &byte);
    // Convert token to bitset
    std::bitset<2> status_of_digital_inputs_{ byte };
    // Parse unused token
    details::parse_token(&scan_data_strtok_save_ptr);

    LOG_DEBUG(" - status of digital inputs: " << status_of_digital_inputs_);

    return status_of_digital_inputs_;
}


LMS1xx::ScanData::DigitalOutputsStatus ScanDataParser::parse_status_of_digital_outputs() const {

    uint8_t low_byte;
    uint8_t high_byte;

    // Parse tokens
    details::parse_token_with_args(&scan_data_strtok_save_ptr, "%hhX", &low_byte);
    details::parse_token_with_args(&scan_data_strtok_save_ptr, "%hhX", &high_byte);
    // Convert token to bitset
    LMS1xx::ScanData::DigitalOutputsStatus status_of_digital_outputs { .internal{ low_byte }, .external{ high_byte } };

    LOG_DEBUG(" - status of digital outputs:");
    LOG_DEBUG("   - internal:" << status_of_digital_outputs.internal);
    LOG_DEBUG("   - external:" << status_of_digital_outputs.external);

    return status_of_digital_outputs;
}

/* ============================================= Private methods (scans info parsing) ============================================= */

LMS1xx::ScanData::LayerAngle ScanDataParser::parse_layer_angle() const {

    uint16_t layer_angle;

    // Parse token
    details::parse_token_with_args(&scan_data_strtok_save_ptr, "%hX", &layer_angle);

    LOG_DEBUG(" - layer angle: 0x" << std::hex << int(layer_angle));

    // Convert layer angle to enumeration
    switch(layer_angle) {
        case 0x0:    return LMS1xx::ScanData::LayerAngle::Layer2; break;
        case 0xFF06: return LMS1xx::ScanData::LayerAngle::Layer3; break;
        case 0xFA:   return LMS1xx::ScanData::LayerAngle::Layer1; break;
        case 0xFE0C: return LMS1xx::ScanData::LayerAngle::Layer4; break;
        default: /* Should not happen */
            LOG_WARNING("Invalid layer angle received (" << layer_angle << ")");
    }

    return to_enum<LMS1xx::ScanData::LayerAngle>(layer_angle);
}


LMS1xx::ScanData::ScanFrequency ScanDataParser::parse_scan_frequency() const {

    uint32_t scan_frequency;

    // Parse token
    details::parse_token_with_args(&scan_data_strtok_save_ptr, "%X", &scan_frequency);

    LOG_DEBUG(" - scan frequency: " << std::dec << (scan_frequency / 100) << " [Hz]");


    // Convert status to enumeration
    switch(scan_frequency) {
        case 2500: return LMS1xx::ScanData::ScanFrequency::Freq25Hz; break;
        case 5000: return LMS1xx::ScanData::ScanFrequency::Freq50Hz; break;
        default: /* Should not happen */
            LOG_WARNING("Invalid scan frequency received (" << scan_frequency << " / 100" << ")");
    }

    return to_enum<LMS1xx::ScanData::ScanFrequency>(scan_frequency);
}


uint32_t ScanDataParser::parse_measurements_frequency() const {

    uint32_t measurements_frequency;

    // Parse token
    details::parse_token_with_args(&scan_data_strtok_save_ptr, "%X", &measurements_frequency);

    LOG_DEBUG(" - measurements frequency: " << std::dec << measurements_frequency << " [Hz]");

    return measurements_frequency;
}

/* ============================================ Private methods (encoders info parsing) =========================================== */

void ScanDataParser::parse_encoders_info(std::vector<LMS1xx::ScanData::EncoderInfo> &info) const {

    uint16_t amount_of_encoders;

    // Parse amount of encoders token
    details::parse_token_with_args(&scan_data_strtok_save_ptr, "%hX", &amount_of_encoders);

    LOG_DEBUG(" - amount of encoders: " << std::dec << amount_of_encoders);

    // Resize output vector
    info.resize(static_cast<std::size_t>(amount_of_encoders));
    // Parse encoders info
    for(int i = 0; i < amount_of_encoders; ++i) {

        // Parse position token
        details::parse_token_with_args(&scan_data_strtok_save_ptr, "%X", &(info[i].position));
        // Parse speed token
        details::parse_token_with_args(&scan_data_strtok_save_ptr, "%hX", &(info[i].speed));
    }
}

/* ================================================ Private methods (data parsing) ================================================ */

namespace details {

    /**
     * @brief Auxiliary function parsing data channel
     * 
     * @retval true 
     *    if number of parsed channels is non-0
     * @retval false 
     *    otherwise
     */
    template<
        typename InitialLoggerType,
        typename ParseType
    > static inline void parse_data_channel(
        rclcpp::Logger *logger,
        char **scan_data_strtok_save_ptr,
        LMS1xx::ScanData::DataSet &channel_data,
        details::ScanDataFlagSet &parsed_content_flags,
        InitialLoggerType &&initial_log,
        ParseType &&parse
    ) {
        
        int number_of_channels;

        // Parse 'Amount of X bit channels' field
        details::parse_token_with_args(scan_data_strtok_save_ptr, "%d", &number_of_channels);

        initial_log(number_of_channels);

        // Parse data from subsequent channels
        for (int i = 0; i < number_of_channels; ++i) {

            LOG_DEBUG("   - channel " << i << ": ");

            char content_bytes[6];

            // Parse 'Content' subfield of the output channel field
            details::parse_token_with_args(scan_data_strtok_save_ptr, "%s", content_bytes);
            // Wrap content type string into string view
            std::string_view content_view { content_bytes };

            LMS1xx::ScanData::ChannelContentType content;

            // Parse content of the channel
                 if(content_view == "DIST1") content = LMS1xx::ScanData::ChannelContentType::Dist1;
            else if(content_view == "DIST2") content = LMS1xx::ScanData::ChannelContentType::Dist2;
            else if(content_view == "RSSI1") content = LMS1xx::ScanData::ChannelContentType::Rssi1;
            else if(content_view == "RSSI2") content = LMS1xx::ScanData::ChannelContentType::Rssi2;
            else /* Should not happen */ 
                assert(false);

            LOG_DEBUG("     * content type: " << content_view );

            // If data of the given content has been already parsed, continue
            if(parsed_content_flags[content]) {
                LOG_WARNING("     * skipping channel... (data of this type already parsed) [this should not happen!]");
                continue;
            // Else mark data as parsed
            } else
                parsed_content_flags[content] = true;

            // If data of this type has not entry in the output buffer, initialize it
            if(not channel_data[content].has_value())
                channel_data[content].emplace();
            // Get handle to the output data entry
            auto &channel_data_entry = *channel_data[content];

            // Parse 'Scale factor' subfield of the output channel field
            details::parse_token_with_args(scan_data_strtok_save_ptr, "%X", &channel_data_entry.scale_factor);
            // Parse 'Scale factor offset' subfield of the output channel field
            details::parse_token_with_args(scan_data_strtok_save_ptr, "%X", &channel_data_entry.scale_factor_offset);
            // Parse 'Start angle' subfield of the output channel field
            int32_t start_angle;
            details::parse_token_with_args(scan_data_strtok_save_ptr, "%X", &start_angle);
            channel_data_entry.start_angle_rad = lidar_angle_to_rad(start_angle);
            // Parse 'Size of single angular step' subfield of the output channel field
            uint16_t size_of_single_angular_step;
            details::parse_token_with_args(scan_data_strtok_save_ptr, "%hX", &size_of_single_angular_step);
            channel_data_entry.angular_step_size_rad = lidar_angle_to_rad(size_of_single_angular_step);

            LOG_DEBUG("     * scale factor: "                << channel_data_entry.scale_factor                     );
            LOG_DEBUG("     * scale factor offset: "         << channel_data_entry.scale_factor_offset              );
            LOG_DEBUG("     * start angle: "                 << channel_data_entry.start_angle_rad       << " [rad]");
            LOG_DEBUG("     * size of single angular step: " << channel_data_entry.angular_step_size_rad << " [rad]");

            uint16_t amount_of_data;

            // Parse 'Amount of data' field
            details::parse_token_with_args(scan_data_strtok_save_ptr, "%hX", &amount_of_data);
            
            LOG_DEBUG("     * amount of data: " << amount_of_data);

            // Resize data buffer
            channel_data_entry.data.resize(amount_of_data);

            // Check if data is of a distance type
            bool is_distance_data = 
                (content == LMS1xx::ScanData::ChannelContentType::Dist1) or 
                (content == LMS1xx::ScanData::ChannelContentType::Dist2);

            // Auxiliary data parsing routine
            auto parse_data = [&](int data_idx) {
                
                int data;

                // Parse data halfword
                details::parse_token_with_args(scan_data_strtok_save_ptr, "%X", &data);
                // Copy data halfword to the output buffer
                parse(channel_data_entry.data[data_idx], data);
            };

            // Parse data (distance)
            if(is_distance_data) {
                for (int j = 0; j < amount_of_data; ++j) {
                    
                    // Parse data
                    parse_data(j);
                    // Scale distance down from [mm] to [m]
                    channel_data_entry.data[j] = (channel_data_entry.data[j] / 1'000.0);

                }
            // Parse data (intensities)
            } else {
                for (int j = 0; j < amount_of_data; ++j) {

                    // Parse data
                    parse_data(j);

                }
            }
        }
    };

}


void ScanDataParser::parse_channel_16_bit_data(
    LMS1xx::ScanData::DataSet &channel_data,
    details::ScanDataFlagSet &parsed_content_flags
) const {
    details::parse_data_channel(
        /* Logger            */ logger,
        /* strtok_r() buffer */ &scan_data_strtok_save_ptr,
        /* Data              */ channel_data,
        /* Parsed data flags */ parsed_content_flags,
        /* Initial logger    */ [this](auto number_of_channels     ) { LOG_DEBUG(" - number of 16-bit channels: " << number_of_channels); },
        /* Parser            */ [this](auto &buffer_entry, int data) { buffer_entry = static_cast<uint16_t>(data);                        }
    );
}


void ScanDataParser::parse_channel_8_bit_data(
    LMS1xx::ScanData::DataSet &channel_data,
    details::ScanDataFlagSet &parsed_content_flags
) const {
    details::parse_data_channel(
        /* Logger            */ logger,
        /* strtok_r() buffer */ &scan_data_strtok_save_ptr,
        /* Data              */ channel_data,
        /* Parsed data flags */ parsed_content_flags,
        /* Initial logger    */ [this](auto number_of_channels     ) { LOG_DEBUG(" - number of 8-bit channels: " << number_of_channels); },
        /* Parser            */ [this](auto &buffer_entry, int data) { buffer_entry = static_cast<uint8_t>(data);                        }
    );
}

void ScanDataParser::parse_channel_data(LMS1xx::ScanData::DataSet &channel_data) const {

    // Zero-initialize set of flags indicating data of what content has been parsed
    details::ScanDataFlagSet content_has_been_parsed { };

    // Parse 16-bit data
    parse_channel_16_bit_data(channel_data, content_has_been_parsed);
    // Parse 8-bit data
    parse_channel_8_bit_data(channel_data, content_has_been_parsed);

    // Erase channel data structures of types that has not been parsed in this call
    for(std::size_t i = 0; i < content_has_been_parsed.size(); ++i) {
        if(not content_has_been_parsed[i])
            channel_data[i].reset();
    }

}

/* =========================================== Private methods (auxiliary info parsing) =========================================== */

void ScanDataParser::parse_name() const {

    // Parse unused position-present flag token
    details::parse_token(&scan_data_strtok_save_ptr);

    int name_present;

    // Parse name-present flag token
    details::parse_token_with_args(&scan_data_strtok_save_ptr, "%d", &name_present);

    LOG_DEBUG(" - name present: " << (name_present == 1));

    // If name present, skip its fields
    if(name_present == 1)
        details::parse_nth_token(2, &scan_data_strtok_save_ptr);
}


void ScanDataParser::parse_comment() const {

    int comment_present;

    // Parse name-present flag token
    details::parse_token_with_args(&scan_data_strtok_save_ptr, "%d", &comment_present);

    LOG_DEBUG(" - comment present: " << (comment_present == 1));

    // If name present, skip its fields
    if(comment_present == 1)
        details::parse_nth_token(2, &scan_data_strtok_save_ptr);

}


std::optional<LMS1xx::ScanData::TimestampInfo> ScanDataParser::parse_timestamp_info() const {
    
    int time_present;


    /// Parse 'Time' field token
    details::parse_token_with_args(&scan_data_strtok_save_ptr, "%d", &time_present);
    
    LOG_DEBUG(" - time present: " << (time_present == 1));

    // If time data is output
    if(time_present == 1){
        
        LMS1xx::ScanData::TimestampInfo ret;

        // Zero-initialize time struct
        struct tm msg_time = { 0 };
        
        uint16_t year;

        // Parse [year]
        details::parse_token_with_args(&scan_data_strtok_save_ptr, "%hX", &year);
        // Insert [year] into the time structure
        msg_time.tm_year = (int) year - 1900;

        uint8_t data_byte;

        // Parse [month]
        details::parse_token_with_args(&scan_data_strtok_save_ptr, "%hhX", &data_byte);
        // Insert [month] into the time structure
        msg_time.tm_mon = (int) data_byte - 1;

        // Parse [day]
        details::parse_token_with_args(&scan_data_strtok_save_ptr, "%hhX", &data_byte);
        // Insert [day] into the time structure
        msg_time.tm_mday = (int) data_byte;

        // Parse [hour]
        details::parse_token_with_args(&scan_data_strtok_save_ptr, "%hhX", &data_byte);
        // Insert [hour] into the time structure
        msg_time.tm_hour = (int) data_byte;

        // Parse [minute]
        details::parse_token_with_args(&scan_data_strtok_save_ptr, "%hhX", &data_byte);
        // Insert [minute] into the time structure
        msg_time.tm_min = (int) data_byte;

        // Parse [second]
        details::parse_token_with_args(&scan_data_strtok_save_ptr, "%hhX", &data_byte);
        // Insert [second] into the time structure
        msg_time.tm_sec = (int) data_byte;

        uint32_t microseconds;

        // Parse [microsecond]
        details::parse_token_with_args(&scan_data_strtok_save_ptr, "%X", &microseconds);

        // Parse timestamp into the output structure
        ret.seconds      = std::chrono::seconds     { mktime(&msg_time) };
        ret.microseconds = std::chrono::microseconds{ microseconds      };

        LOG_DEBUG("   - year: "                  << 1900 + msg_time.tm_year  );
        LOG_DEBUG("   - month: "                 << msg_time.tm_mon          );
        LOG_DEBUG("   - day: "                   << msg_time.tm_mday         );
        LOG_DEBUG("   - hour: "                  << msg_time.tm_hour         );
        LOG_DEBUG("   - minute: "                << msg_time.tm_min          );
        LOG_DEBUG("   - second: "                << msg_time.tm_sec          );
        LOG_DEBUG("   - microsecond: "           << ret.microseconds.count() );
        LOG_DEBUG("   - seconds (since epoch): " << ret.seconds.count()      );

        return ret;
    }

    return std::optional<LMS1xx::ScanData::TimestampInfo>{ };
}

/* ======================================================== Public methods ======================================================== */

void ScanDataParser::parse(char *data_buffer, LMS1xx::ScanData &parsed_data) const {

    LOG_DEBUG("Parsing scan data...");

    /* ----------------------- Header parsing ------------------------ */

    parse_header(data_buffer);

    /* --------------------- Device info parsing ---------------------- */

    parsed_data.version_number = parse_version_number();
    parsed_data.device_number  = parse_device_number();
    parsed_data.serial_number  = parse_serial_number();
    parsed_data.device_status  = parse_device_status();

    /* ------------------ Communication info parsing ------------------ */

    parsed_data.telegram_counter        = parse_telegram_counter();
    parsed_data.scans_counter           = parse_scans_counter();
    parsed_data.time_since_start_us     = parse_time_since_start_us();
    parsed_data.time_of_transmission_us = parse_time_of_transmission_us();

    /* ---------------------- Digital I/O parsing --------------------- */

    parsed_data.status_of_digital_inputs  = parse_status_of_digital_inputs();
    parsed_data.status_of_digital_outputs = parse_status_of_digital_outputs();

    /* ---------------------- Scans info parsing ---------------------- */

    parsed_data.layer_angle            = parse_layer_angle();
    parsed_data.scan_frequency         = parse_scan_frequency();
    parsed_data.measurements_frequency = parse_measurements_frequency();

    /* -------------------- Encoders info parsing -------------------- */

    parse_encoders_info(parsed_data.encoders_info);

    /* ------------------------- Data parsing ------------------------ */
    
    parse_channel_data(parsed_data.channel_data);

    /* -------------------- Auxiliary info parsing ------------------- */

                                 parse_name();
                                 parse_comment();
    parsed_data.timestamp_info = parse_timestamp_info();

}

/* ================================================================================================================================ */

} // End namespace sick::lms1xx

/* ================================================================================================================================ */
