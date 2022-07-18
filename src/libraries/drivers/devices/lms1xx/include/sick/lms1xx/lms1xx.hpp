/* ============================================================================================================================ *//**
 * @file       lms1xx.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Monday, 4th July 2022 3:58:11 pm
 * @modified   Thursday, 7th July 2022 7:20:08 pm
 * @project    engineering-thesis
 * @brief      Implementation of inline methods of the LMS1xx class
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __SICK_LMS1XX_LMS1XX_H__
#define __SICK_LMS1XX_LMS1XX_H__

/* =========================================================== Includes =========================================================== */

// Standard includes
#include <cstdio>
// Linux includes
#include <unistd.h>
// Private includes
#include "sick/lms1xx.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace sick {

/* ======================================================= Auxiliary macros ======================================================= */

/// Logs debug message in a stream-like fashion, if logger is configrued
#define LOG_DEBUG(...)    \
    if(logger.has_value()) \
        RCLCPP_DEBUG_STREAM(*logger, "[" << lms1xx::context_base << "::" << context << "] " << __VA_ARGS__)

/// Logs warning message in a stream-like fashion, if logger is configrued
#define LOG_WARNING(...)    \
    if(logger.has_value()) \
        RCLCPP_WARN_STREAM(*logger, "[" << lms1xx::context_base << "::" << context << "] " << __VA_ARGS__)

/* ====================================================== Auxiliary constants ===================================================== */

namespace lms1xx {

    /// Context string for log messages
    static constexpr auto context_base = "sick::LMS1xx";
    
}

/* ====================================================== Auxiliary functions ===================================================== */

namespace details {

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

}

/* ========================================================= Public ctors ========================================================= */

LMS1xx::LMS1xx(std::string_view host) :
    host{ host }
{ }


LMS1xx::LMS1xx(
    rclcpp::Logger logger,
    std::string_view host
) :
    logger{ logger },
    host{ host },
    buffer{ *(this->logger) }
{ }


LMS1xx::~LMS1xx() {

    // Command LIDAR to stop measurements
    stop_measurements();

    // Disconnect host from the network
    disconnect();
    
}

/* ========================================================= Nested types ========================================================= */

std::optional<LMS1xx::ScanData::Data> &LMS1xx::ScanData::DataSet::operator[](ChannelContentType type) {
    return base_type::operator[](static_cast<std::size_t>(type));
}

/* ============================================= Public methods (connection interface) ============================================ */

void LMS1xx::disconnect() {

    static constexpr auto context = "disconnect";

    // If driver has connected to the LIDAR
    if(socket_fd.has_value()) {

        // Close the socket
        ::close(*socket_fd);
        
        // Remove current descriptor
        socket_fd.reset();

        LOG_DEBUG("Driver disconnected");
    }

}


bool LMS1xx::is_connected() const {
    return socket_fd.has_value();
}

/* ============================================ Public methods (measurements interface) =========================================== */

void LMS1xx::start_measurements() {

    static constexpr auto context = "LMS1xx::start_measurements";

    // Create I/O helper manager
    auto io = make_io_manager(context);
    // Write command to the device
    io.exchange("sMN LMCstartmeas");
}


void LMS1xx::stop_measurements() {

    static constexpr auto context = "LMS1xx::stop_measurements";

    // Create I/O helper manager
    auto io = make_io_manager(context);
    // Write command to the device
    io.exchange("sMN LMCstopmeas");
}


LMS1xx::Status LMS1xx::get_status() const {

    static constexpr auto context = "LMS1xx::get_status";

    // Create I/O helper manager
    auto io = make_io_manager(context);
    // Write command to the device
    io.exchange("sRN STlms");

    int status;

    // Parse status from the response
    sscanf(&io.get_response()[9], "%d", &status);

    return details::to_enum<Status>(status);
}


void LMS1xx::enable_continous_scan(bool enable) {

    static constexpr auto context = "LMS1xx::enable_continous_scan";

    // Create I/O helper manager
    auto io = make_io_manager(context);
    // Write command to the device
    io.exchange("%s %d", "sEN LMDscandata", enable ? 1 : 0);
}

/* =========================================== Public methods (configruation interface) =========================================== */

void LMS1xx::save_current_config() {

    static constexpr auto context = "LMS1xx::save_current_config";

    // Create I/O helper manager
    auto io = make_io_manager(context);
    // Exchange packet with the device
    io.exchange("sMN mEEwriteall");
}


void LMS1xx::logout() {

    static constexpr auto context = "LMS1xx::logout";

    // Create I/O helper manager
    auto io = make_io_manager(context);
    // Exchange packet with the device
    io.exchange("sMN Run");
}

/* ================================================== Public methods (utilities) ================================================== */

constexpr std::string_view LMS1xx::to_str(SetScanConfigStatus status) {
    switch(status) {
        case SetScanConfigStatus::Ok:                         return "Ok";
        case SetScanConfigStatus::FrequencyError:             return "Frequency error";
        case SetScanConfigStatus::ResolutionError:            return "Resolution error";
        case SetScanConfigStatus::ResolutionAndScanAreaError: return "Resolution and scan area error";
        case SetScanConfigStatus::ScanAreaError:              return "Scan area error";
        case SetScanConfigStatus::OtherError:                 return "Other error";
        default:
            return "<Unknown>";
    }
}

/* ======================================================= Protected methods ====================================================== */

lms1xx::IOManager LMS1xx::make_io_manager(std::string_view log_context) const {
    return lms1xx::IOManager{ socket_fd, logger.has_value() ? &(*logger) : nullptr, log_context };
}


lms1xx::ScanDataParser LMS1xx::make_scan_data_parser() const {
    return lms1xx::ScanDataParser{ logger.has_value() ? &(*logger) : nullptr };
}

/* ============================================================ Cleanup =========================================================== */

#undef LOG_DEBUG
#undef LOG_WARNING

/* ================================================================================================================================ */

} // End namespace sick

/* ================================================================================================================================ */

#endif
