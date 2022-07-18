/* ============================================================================================================================ *//**
 * @file       lms1xx.cpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Monday, 4th July 2022 3:58:11 pm
 * @modified   Thursday, 7th July 2022 6:50:29 pm
 * @project    engineering-thesis
 * @brief      Implementation of methods of the LMS1xx class
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */


/* =========================================================== Includes =========================================================== */

// Standard includes
#include <type_traits>
#include <cstdio>
// Linux includes
#include <unistd.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
// Private includes
#include "sick/lms1xx.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace sick {

/* ======================================================= Auxiliary macros ======================================================= */

/// Logs debug message in a stream-like fashion, if logger is configrued
#define LOG_DEBUG(...)     \
    if(logger.has_value()) \
        RCLCPP_DEBUG_STREAM(*logger, "[" << lms1xx::context_base << "::" << context << "] " << __VA_ARGS__)

/// Logs warning message in a stream-like fashion, if logger is configrued
#define LOG_WARNING(...)   \
    if(logger.has_value()) \
        RCLCPP_WARN_STREAM(*logger, "[" << lms1xx::context_base << "::" << context << "] " << __VA_ARGS__)

/* ====================================================== Auxiliary functions ===================================================== */

namespace details {

    /// Converts @p angle from [1/10'000 deg] to [rad]
    template<typename T>
    double lidar_angle_to_rad(T angle) {
        return ((static_cast<double>(angle) * 2 * M_PI / 10'000.0) / 360.0);
    }


    /// Converts @p angle from [rad] to [1/10'000 deg]
    uint32_t rad_to_lidar_angle(double angle) {
        return static_cast<uint32_t>(angle / (2 * M_PI) * 360.0 * 10'000.0);
    }


    /// Converts @p angle from [rad] to [1/10'000 deg]
    int32_t rad_to_lidar_angle_signed(double angle) {
        return static_cast<int32_t>(angle / (2 * M_PI) * 360.0 * 10'000.0);
    }

}

/* ============================================= Public methods (connection interface) ============================================ */

void LMS1xx::connect() {

    static constexpr auto context = "connect";

    // If driver has been connected yet, return
    if(socket_fd.has_value())
        return;
        
    LOG_DEBUG("Creating non-blocking socket...");

    // Create the UDP socket
    auto fd = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);

    // If succeeded to create the socker
    if (fd != 0) {

        struct sockaddr_in sock_addr_desc;

        // Prepare description of the target UDP port (ASCII interface)
        sock_addr_desc.sin_family = PF_INET;
        sock_addr_desc.sin_port   = htons(SICK_COLA_ASCII_INTERFACE_PORT);
        // Parse IP address
        inet_pton(AF_INET, host.c_str(), &sock_addr_desc.sin_addr);

        LOG_DEBUG("Connecting socket to laser at " << host << ":" << SICK_COLA_ASCII_INTERFACE_PORT << "...");

        // Connect to the socket
        if (::connect(fd, (struct sockaddr *) &sock_addr_desc, sizeof(sock_addr_desc)) == 0) {

            // Keep socket descriptor
            socket_fd = fd;
            
            LOG_DEBUG("Connected succeeded (descriptor: " <<fd << ")");
        
        // On failure, log warning
        } else 
            LOG_WARNING("Failed to connect to the LASER");

    // On error, log warning
    } else 
        LOG_WARNING("Failed to create the socket");;

    return;
}

/* ============================================ Public methods (measurements interface) =========================================== */


void LMS1xx::get_scan_data(ScanData &data) const {

    static constexpr auto context = "get_scan_data";

    fd_set rfds;

    // Zero-initialize descriptors set for select() cakk
    FD_ZERO(&rfds);
    // Set socket-related flag
    FD_SET(*socket_fd, &rfds);

    // Block a total of up to 100 [ms] waiting for more data from the laser.
    while(true) {

        struct timeval tv;
        
        // Set time slice to 100 [ms]
        tv.tv_sec  = 0;
        tv.tv_usec = 100'000;

        /*
         * @note Would be great to depend on linux's behaviour of updating the timeval, 
         *   but unfortunately that's non-POSIX (doesn't work on OS X, for example)
         */

        LOG_DEBUG("Entering select " << (tv.tv_usec / 1'000) << " [ms]");

        // Wait for I/O
        int retval = select(
            /* FDs num to be tested in the set */ *socket_fd + 1,
            /* FDs set (read)                  */ &rfds,
            /* FDs set (write)                 */ NULL,
            /* FDs set (except)                */ NULL,
            /* Timtoue                         */ &tv
        );

        LOG_DEBUG("Returned " << retval <<  " from select()");

        // On I/O success
        if (retval > 0) {

            // Read scan data from the socket
            buffer.read(*socket_fd);
            
            // Try to parse scan data message from the buffer 
            uint8_t* buffer_data = buffer.get_next_message();
            
            // If full scan data message has been received, parse it
            if(buffer_data != nullptr) {

                auto parser = make_scan_data_parser();

                // Parse scan data
                parser.parse(reinterpret_cast<char*>(buffer_data), data);
                // Remove parsed message from the buffer
                buffer.pop_last_buffer();
                
                return;
            }

            /*
             * If scan data could not be parsed it means that only part of the message has been
             * received. In such a case continue loop to parse rest of data
             */
        
        // If select() timed out or there was an fd error, throw error
        } else {
            std::stringstream ss;
            ss << "[" << lms1xx::context_base << "::" << context << "] Failed to read scan data from the socket" ;
            throw std::runtime_error{ ss.str() };
        }
    }
}

/* ================================================ Public methods (NTP interface) ================================================ */

void LMS1xx::set_ntp_config(const NtpConfig &cfg) {

    static constexpr auto context = "set_ntp_config";

    /* ------------------ Verify configuration ------------------- */

    bool valid_ip = 
        (cfg.server_ip[0] >= 0) and (cfg.server_ip[0] <= 255) and
        (cfg.server_ip[1] >= 0) and (cfg.server_ip[1] <= 255) and
        (cfg.server_ip[2] >= 0) and (cfg.server_ip[2] <= 255) and
        (cfg.server_ip[3] >= 0) and (cfg.server_ip[3] <= 255);
        
    // If invalid IP given, throw
    if (not valid_ip) {
        std::stringstream ss;
        ss << "[" << lms1xx::context_base << "::" << context << "] Invalid IP address";
        throw std::runtime_error{ ss.str() };
    }

    bool valid_timezone = 
        (cfg.time_zone.count() >= -12) and (cfg.time_zone.count() <= 12);
        
    // If invalid timezone given, throw
    if (not valid_timezone) {
        std::stringstream ss;
        ss << "[" << lms1xx::context_base << "::" << context << "] Invalid timezone given (" << cfg.time_zone.count() << ")";
        throw std::runtime_error{ ss.str() };
    }

    bool valid_update_time = 
        (cfg.update_time.count() >= 1) and (cfg.update_time.count() <= 3600);
        
    // If invalid update time, throw
    if (not valid_update_time) {
        std::stringstream ss;
        ss << "[" << lms1xx::context_base << "::" << context << "] Invalid update time given (" << cfg.update_time.count() << ")";
        throw std::runtime_error{ ss.str() };
    }

    /* ---------------- Write down configuration ----------------- */

    auto io = make_io_manager("LMS1xx::set_ntp_config");

    // Set NTP role
    io.exchange("%s %d", "sWN TSCRole", details::to_underlying(cfg.role));
    // Set time synchronization interface command 
    io.exchange("%s %d", "sWN TSCTCInterface", details::to_underlying(cfg.time_sync_interface));
    // Set time server IP address command 
    io.exchange("%s %X %X %X %X", "sWN TSCTCSrvAddr", cfg.server_ip[0], cfg.server_ip[1], cfg.server_ip[2], cfg.server_ip[3]);
    // Set timezone command 
    io.exchange("%s %X", "sWN TSCTCtimezone", cfg.time_zone.count() + 12);
    // Set timezone command 
    io.exchange("%s +%d", "sWN TSCTCupdatetime", static_cast<int>(cfg.update_time.count()));
}


LMS1xx::NtpStatus LMS1xx::get_ntp_status() const {

    NtpStatus ret;

    // Make I/O interface
    auto io = make_io_manager("LMS1xx::get_ntp_config");
    // Prepare auxiliary functor to convert time from LIDAR format to C++ entity
    auto time_to_chrono = [](const uint32_t bytes) {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<float, std::ratio<1>>(*((float*) &bytes))
        ); 
    };

    /* --------------------- Read max offset --------------------- */

    // Write command to the device and receive response
    io.exchange("sRN TSCTCmaxoffset");

    uint32_t offset_bytes;

    // Parse response (seconds in standard floating-point format)
    sscanf(&io.get_response()[19], "%X", &offset_bytes);

    // Reinterpret data and convert from [s] to [ns]
    ret.max_offset_ntp_ns = time_to_chrono(offset_bytes);

    /* ------------------------ Read delay ----------------------- */

    // Write command to the device and receive response
    io.exchange("sRN TSCTCdelay");

    uint32_t delay_bytes;

    // Parse response (seconds in standard floating-point format)
    sscanf(&io.get_response()[15], "%X", &delay_bytes);

    // Reinterpret data and convert from [s] to [ns]
    ret.time_delay_ns = time_to_chrono(delay_bytes);

    /* ----------------------------------------------------------- */

    return ret;
}

/* =========================================== Public methods (configruation interface) =========================================== */

void LMS1xx::login() {

    // Create I/O helper manager
    auto io = make_io_manager("LMS1xx::login");
    // Format login command
    io.format_cmd("sMN SetAccessMode 03 F4724744");

    int result;

    // Loop until data is available to read
    do {
        
        timeval timeout;

        // Prepare timeout 1 [s]
        timeout.tv_sec  = 1;
        timeout.tv_usec = 0;

        // Write 
        io.write();
        // Wait for response
        result = io.select(timeout);

    } while (result <= 0);

    // Read response
    io.read();
}


LMS1xx::ScanConfig LMS1xx::get_scan_config() const {

    static constexpr auto context = "get_scan_config";

    /* ---------------------------- Perfomr I/O ---------------------------- */

    // Create I/O helper manager
    auto io = make_io_manager("LMS1xx::get_scan_config");
    // Format command
    io.exchange("sRN LMPscancfg");

    uint32_t scaning_frequency;
    uint32_t angular_resolution;
    int32_t start_angle;
    int32_t stop_angle;

    // Parse response
    sscanf(
        io.get_response().data(),
        /* Command type       */ "%*s "
        /* Command            */ "%*s "
        /* Scan frequency     */ "%X "
        /* Number of sensors  */ "%*d "
        /* Angular resolution */ "%X "
        /* Start angle        */ "%X "
        /* Stop angle         */ "%X",
        &scaning_frequency,
        &angular_resolution,
        reinterpret_cast<uint32_t*>(&start_angle),
        reinterpret_cast<uint32_t*>(&stop_angle)
    );

    /* -------------------------- Parse response --------------------------- */

    ScanConfig ret;

    // Parse scanning frequency
    switch(scaning_frequency) {
        case 2500: ret.scaning_frequency_hz = ScanConfig::ScanFrequency::Freq25Hz; break;
        case 5000: ret.scaning_frequency_hz = ScanConfig::ScanFrequency::Freq50Hz; break;
        default:
            ret.scaning_frequency_hz = details::to_enum<ScanConfig::ScanFrequency>(scaning_frequency);
            LOG_WARNING("Invalid scanning frequency has been read (" << scaning_frequency << ")");
    }
    // Parse angular resoultion
    switch(angular_resolution) {
        case 2500: ret.angular_step_size = ScanConfig::AngularResolution::Res0_25deg; break;
        case 5000: ret.angular_step_size = ScanConfig::AngularResolution::Res0_50deg; break;
        default:
            ret.angular_step_size = details::to_enum<ScanConfig::AngularResolution>(scaning_frequency);
            LOG_WARNING("Invalid angular resolution has been read (" << angular_resolution << ")");
    }
    // Parse start/stop angles
    ret.start_angle_rad = details::lidar_angle_to_rad(start_angle);
    ret.stop_angle_rad  = details::lidar_angle_to_rad(stop_angle);

    LOG_DEBUG("Received configruation: ");
    LOG_DEBUG(" - scanningfrequency: " << (ret.scaning_frequency_hz == ScanConfig::ScanFrequency::Freq25Hz       ? "25"   : "50")  << " [Hz]" );
    LOG_DEBUG(" - angular step: "      << (ret.angular_step_size    == ScanConfig::AngularResolution::Res0_25deg ? "0.25" : "0.5") << " [deg]");
    LOG_DEBUG(" - start angle: "       << ret.start_angle_rad                                                                      << " [rad]");
    LOG_DEBUG(" - stop angle: "        << ret.stop_angle_rad                                                                       << " [rad]");

    return ret;
}


LMS1xx::SetScanConfigStatus LMS1xx::set_scan_config(const ScanConfig &cfg) {

    static constexpr auto context = "set_scan_config";
    
    /* --------------------------- Parse config ---------------------------- */

    // Create I/O helper manager
    auto io = make_io_manager("LMS1xx::set_scan_config");

    uint32_t scaning_frequency;
    uint32_t angular_resolution;

    // Parse scanning frequency
    switch(cfg.scaning_frequency_hz) {
        case ScanConfig::ScanFrequency::Freq25Hz: scaning_frequency = 2500; break;
        case ScanConfig::ScanFrequency::Freq50Hz: scaning_frequency = 5000; break;
        default:
            LOG_WARNING("Invalid scanning frequency has been given (" << details::to_underlying(cfg.scaning_frequency_hz) << ")");
    }
    // Parse angular resoultion
    switch(cfg.angular_step_size) {
        case ScanConfig::AngularResolution::Res0_25deg: angular_resolution = 2500; break;
        case ScanConfig::AngularResolution::Res0_50deg: angular_resolution = 5000; break;
        default:
            LOG_WARNING("Invalid angular resolution has been given (" << details::to_underlying(cfg.angular_step_size) << ")");
    }

    /* ---------------------------- Perfomr I/O ---------------------------- */
    
    // Format login command
    io.exchange(
        /* Command type & Command   */ "%s "
        /* Scan frequency           */ "%X "
        /* Number of active sensors */ "+1 "
        /* Angular resolution       */ "%X "
        /* Start angle              */ "-450000 "  // Only one value valid for LMS1xx
        /* Stop angle               */ "+2250000", // Only one value valid for LMS1xx
        "sMN mLMPsetscancfg",
        scaning_frequency,
        angular_resolution
    );

    int status_code;

    // Parse response
    sscanf(
        io.get_response().data(),
        /* Command type       */ "%*s "
        /* Command            */ "%*s "
        /* Number of sensors  */ "%d ",
        &status_code
    );

    LOG_DEBUG("Status: " << to_str(static_cast<SetScanConfigStatus>(status_code)));

    return static_cast<SetScanConfigStatus>(status_code);
}



void LMS1xx::set_scan_data_config(const ScanDataConfig &cfg) {

    // Create I/O helper manager
    auto io = make_io_manager("LMS1xx::set_scan_data_config");

    // Format command
    io.exchange(
        /* Command type & command */ "%s "
        /* Data channel           */ "%d 0 "
        /* Remission              */ "%d "
        /* Resolution             */ "%d "
        /* Unit                   */ "0 "
        /* Encoder                */ "%d 0 "
        /* Position               */ "%d "
        /* Device name            */ "%d "
        /* Comment                */ "0 "
        /* Time                   */ "%d "
        /* Output rate            */ "+%d",
        "sWN LMDscandatacfg",
        (details::to_underlying(cfg.output_channel)         )        ,
        (cfg.output_remission                               ) ? 1 : 0,
        (cfg.resolution == ScanDataConfig::Resolution::Bit16) ? 1 : 0,
        (cfg.output_encoder                                 ) ? 1 : 0,
        (cfg.output_position                                ) ? 1 : 0,
        (cfg.output_device_name                             ) ? 1 : 0,
        (cfg.output_timestamp                               ) ? 1 : 0,
        (cfg.output_interval                                )
    );

    return;
}


LMS1xx::ScanOutputRange LMS1xx::get_scan_output_range() const {

    static constexpr auto context = "get_scan_output_range";

    /* ---------------------------- Perfomr I/O ---------------------------- */

    // Create I/O helper manager
    auto io = make_io_manager("LMS1xx::get_scan_data_config");
    // Format command
    io.exchange("sRN LMPscancfg");

    uint32_t angular_resolution;
    int32_t start_angle;
    int32_t stop_angle;

    // Parse response
    sscanf(
        io.get_response().data(),
        /* Command type       */ "%*s "
        /* Command            */ "%*s "
        /* Scan frequency     */ "%*d "
        /* Number of sectors  */ "%*d "
        /* Angular resolution */ "%X "
        /* Start angle        */ "%X "
        /* Stop angle         */ "%X",
        &angular_resolution,
        reinterpret_cast<uint32_t*>(&start_angle),
        reinterpret_cast<uint32_t*>(&stop_angle)
    );

    /* -------------------------- Parse response --------------------------- */

    ScanOutputRange ret;

    // Parse angular resolution
    ret.resolution_rad = details::lidar_angle_to_rad(angular_resolution);
    // Parse start/stop angles
    ret.start_angle_rad = details::lidar_angle_to_rad(start_angle);
    ret.stop_angle_rad  = details::lidar_angle_to_rad(stop_angle);

    LOG_DEBUG("Received configuration:");
    LOG_DEBUG(" - resolution angle: " << ret.resolution_rad  << " [rad]");
    LOG_DEBUG(" - start angle: "      << ret.start_angle_rad << " [rad]");
    LOG_DEBUG(" - stop angle: "       << ret.stop_angle_rad  << " [rad]");

    return ret;

}

/* ================================================================================================================================ */

} // End namespace sick

/* ================================================================================================================================ */
