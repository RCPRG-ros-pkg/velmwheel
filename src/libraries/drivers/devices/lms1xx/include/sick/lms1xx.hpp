/* ============================================================================================================================ *//**
 * @file       lms1xx.hpp
 * @author     Konrad Banachowicz
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Monday, 4th July 2022 2:23:48 pm
 * @modified   Thursday, 7th July 2022 7:20:31 pm
 * @project    engineering-thesis
 * @brief      Definition of the LMS1xx driver class for LMS1xx LIDAR sensors
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __SICK_LMS1XX_H__
#define __SICK_LMS1XX_H__

/* =========================================================== Includes =========================================================== */

// Standard includes
#include <cstdint>
#include <cstdarg>
#include <string>
#include <string_view>
#include <bitset>
// Boost includes
#include <range/v3/span.hpp>
// ROS includes
#include "rclcpp/rclcpp.hpp"
// Private includes
#include "sick/lms1xx/scan_data_buffer.hpp"
#include "sick/lms1xx/io_manager.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace sick {

/* ===================================================== Forward declarations ===================================================== */

namespace lms1xx { class ScanDataParser; }    

/* ============================================================= Class ============================================================ */

/**
 * @brief Class responsible for communicating with LMS1xx device.
 * 
 * @note Currently only the ASCII-based communciation interface is supported
 */
class LMS1xx
{
public: /* ------------------------------------------------ Public constants ------------------------------------------------------ */

    /// UDP port of the LIDAR sensor used for communication with host via ASCII-based interface
    static constexpr uint16_t SICK_COLA_ASCII_INTERFACE_PORT = 2111;

    /// UDP port of the LIDAR sensor used for communication with host via binary interface
    static constexpr uint16_t SICK_COLA_BINARY_INTERFACE_PORT = 2213;

public: /* ------------------------------------------ Public types (process data) ------------------------------------------------- */

    /**
     * @brief Structure of parsed scan data
     */
    struct ScanData {

    public: /* Public types */

        /// Device status enumaration
        enum class DeviceStatus {
            Ok,
            Error,
            PollutionWarning,
            PollutionErrorWithNoDevError,
            PollutionErrorWithDevError,
        };

        /// Digital outputs state
        struct DigitalOutputsStatus {
            
            /// Status of internal outputs
            std::bitset<3> internal;
            /// Status of external outputs
            std::bitset<8> external;

        };
    
        /// Layer angle enumeration
        enum class LayerAngle {
            Layer1,
            Layer2,
            Layer3,
            Layer4
        };

        /// Scan frequency enumeration
        enum class ScanFrequency {
            Freq25Hz,
            Freq50Hz
        };

        /// Encoder-related informations
        struct EncoderInfo {

            /// Position in [ticks]
            uint32_t position;
            /// Speed in [ticks/mm]
            uint16_t speed;
        };

        /// Type of content held by the channel
        enum class ChannelContentType {
            Dist1,
            Dist2,
            Rssi1,
            Rssi2
        };

        /// Number of channel content types
        static constexpr auto CHANNEL_TYPES_NUM = 4;

        /// Structure of the data channel
        struct Data {

            /// Measurements scale factor
            float scale_factor;
            /// Scale factor offset (always 0)
            float scale_factor_offset;

            /// Start angle [rad]
            double start_angle_rad;
            /// Size of the angular step [rad]
            double angular_step_size_rad;
            
            /// Channel's data (distance is scaled to [m])
            std::vector<float> data;
        };

        /// Auxiliary representation of the set of channel data of all types
        struct DataSet : public std::array<std::optional<Data>, CHANNEL_TYPES_NUM> {

            using base_type = std::array<std::optional<Data>, LMS1xx::ScanData::CHANNEL_TYPES_NUM>;

            /// Forward constructors
            using base_type::array;

            /// Additional operator enabling set' indexing with content type enumerations
            inline std::optional<Data> &operator[](ChannelContentType type);
            /// Allow standard indexing usage
            using base_type::operator[];

        };

        /// Parsed timestamp info structure
        struct TimestampInfo {

            /// Seconds since epoch
            std::chrono::seconds seconds;
            /// Microseconds remainer
            std::chrono::microseconds microseconds;

        };

    public: /* Public data */

        /// Version ID
        uint16_t version_number;
        /// Device number
        uint16_t device_number;
        /// Serial number
        uint32_t serial_number;

        /// Status of the device
        DeviceStatus device_status;

        /// Number of measurement telegrams finished in the scanner and given to the interface
        uint16_t telegram_counter;
        /// Number of scans which were crea- ted in the device; counts how many scans were really done
        uint16_t scans_counter;

        /// Counting the time since power up the device; starting with 0
        std::chrono::microseconds time_since_start_us;
        /// Time in μs when the complete scan is transmitted to the buffer for data output; starting with 0
        std::chrono::microseconds time_of_transmission_us;
        
        /// Status of digital inputs
        std::bitset<2> status_of_digital_inputs;
        /// Status of digital outputs
        DigitalOutputsStatus status_of_digital_outputs;

        /// Layer angle
        LayerAngle layer_angle;

        /// Scan frequency
        ScanFrequency scan_frequency;
        /// Measurements frequency (in 100Hz)
        uint32_t measurements_frequency;

        /// Encoders measurements
        std::vector<EncoderInfo> encoders_info;

        /// Data parsed from 16-bit channels
        DataSet channel_data;
        
        /// Timestamp info
        std::optional<TimestampInfo> timestamp_info;
        
    };

public: /* --------------------------------------- Public types (configruation data) ---------------------------------------------- */

    /**
     * @brief Status of the LMS1xx LIDAR sensor
     */
    enum class Status {

        Undefined           = 0,
        Initialisation      = 1,
        Configuration       = 2,
        Idle                = 3,
        Rotated             = 4,
        InPreparation       = 5,
        Ready               = 6,
        ReadyForMeasurement = 7
        
    };

    /**
     * @brief Structure containing NTP role configuration.
     */
    struct NtpConfig {

        /// NTP role
        enum class Role {
            None   = 0,
            Client = 1,
            Server = 2
        } role;

        /// Time synchronization interface.
        enum class TimeSyncInterface {
            Ethernet = 0,
            CAN      = 1
        } time_sync_interface;

        /// Time server IP address (in standard [x.x.x.x] format)
        int server_ip[4];
        
        /**
        * @brief Time zone.
        * @details Set values in number of hours relative to GMT (-12h … +12h)
        */
        std::chrono::hours time_zone;

        /// @brief Update time in seconds
        std::chrono::seconds update_time;
    };

    /**
     * @brief Structure containing NTP status
     */
    struct NtpStatus {

        /// Read maximum offset time
        std::chrono::nanoseconds max_offset_ntp_ns;

        /// Delay time.
        std::chrono::nanoseconds time_delay_ns;
    };

    /**
     * @brief Description of the scan configuration
     */
    struct ScanConfig {

        /// Scan frequency
        using ScanFrequency = ScanData::ScanFrequency;

        /// Scanning frequency.
        ScanFrequency scaning_frequency_hz;

        /// Angular resolution of the scan
        enum class AngularResolution {
            Res0_25deg,
            Res0_50deg
        };

        /// Scanning resolution
        AngularResolution angular_step_size;
        /// Start angle (unused when setting config - LMS1xx has fixed start angle value)
        double start_angle_rad;
        /// Stop angle (unused when setting configuration - LMS1xx has fixed stop angle value)
        double stop_angle_rad;
        
    };

    /**
    * @brief Structure containing scan data configuration.
    */
    struct ScanDataConfig {

        /// Output channels (defines which output channel is activated.)
        enum class OutputChannel {
            First  = 1,
            Second = 2,
            Both   = 3
        } output_channel;

        /// Remission (defines whether remission values are output)
        bool output_remission;

        /// Remission resolution (defines whether the remission values are output with 8-bit or 16bit resolution)
        enum class Resolution {
            Bit8  = 0,
            Bit16 = 1
        } resolution;

        /// Determines whether to output encoders channels with process data
        bool output_encoder;
        /// Determines whether to output position info with process data
        bool output_position;
        /// Determines whether to output device name with process data
        bool output_device_name;
        /// Determines whether to output timestamp info with process data
        bool output_timestamp;

        /**
        * @brief Output interval (defines which scan is output)
        * @details 
        *    01 every scan     \n
        *    02 every 2nd scan \n
        *    ...               \n
        *    50000 every 50000th scan
        */
        uint16_t output_interval;
    };

    /**
    * @brief Structure containing scan output range configuration
    */
    struct ScanOutputRange {

        /// Scanning resolution.
        double resolution_rad;

        /// Start angle.
        double start_angle_rad;
        /// Stop angle
        double stop_angle_rad;
    };

public: /* -------------------------------------------------- Public ctors -------------------------------------------------------- */

    /**
     * @brief Constructs a new LMS1xx object
     * 
     * @param host 
     *    IP of the targe LIDAR
     * 
     * @note This constructor provides no logger to be sued and so no logs will be produces by the class
     */
    inline LMS1xx(std::string_view host);

    /**
     * @brief Constructs a new LMS1xx object
     * 
     * @param logger 
     *    reference to the logger that will be used by the class
     * @param host 
     *    IP of the targe LIDAR
     */
    inline LMS1xx(
        rclcpp::Logger logger,
        std::string_view host
    );

    // Disable copy-construction
    LMS1xx(const LMS1xx& rdriver) = delete;
    // Disable copy-asignment
    LMS1xx &operator=(const LMS1xx& rdriver) = delete;

    // Enable move-construction
    LMS1xx(LMS1xx &&rdriver) = default;
    // Enable move-asignment
    LMS1xx &operator=(LMS1xx &&rdriver) = default;

    /**
     * @brief Destroy the LMS1xx object stopping measurements and disconnecting host from the network
     */
    inline ~LMS1xx();

public: /* -------------------------------------- Public methods (connection interface) ------------------------------------------- */

    /**
     * @brief Connect to LMS1xx.
     * 
     * @param host 
     *    LMS1xx host name or ip address.
     * @param port 
     *    LMS1xx port number.
     */
    void connect();

    /**
     * @brief Disconnect from LMS1xx device.
     */
    inline void disconnect();

    /**
     * @brief Get status of connection.
     * 
     * @returns connected or not.
     */
    inline bool is_connected() const;

public: /* ------------------------------------- Public methods (measurements interface) ------------------------------------------ */

    /**
     * @brief Start measurements
     * @details After receiving this command LMS1xx unit starts spinning laser and measuring.
     */
    inline void start_measurements();

    /**
     * @brief Stop measurements.
     * @details After receiving this command LMS1xx unit stop spinning laser and measuring.
     */
    inline void stop_measurements();

    /**
     * @brief Get current status of LMS1xx device.
     * @returns status of LMS1xx device.
     */
    inline Status get_status() const;

    /**
     * @brief Start or stop continuous data acquisition.
     * @details After reception of this command device start or stop continuous data stream containing scan messages.
     * 
     * @param enable
     *    if @c true , continuous scan will be enabled; otherwise it will be disabled
     */
    inline void enable_continous_scan(bool enable);

    /**
     * @brief Receives single scan message.
     * 
     * @param[out] data
     *    parsed data output buffer (passed by reference to avoid repeated dynaimc allocation
     *    of structure's members)
     * 
     * @throws std::runtime_error
     *    if error or timeout
     * 
     * @note Exception implies that higher level logic should take correct action such as reopening the connection.
     */
    void get_scan_data(ScanData &data) const;

public: /* ------------------------------------------ Public methods (NTP interface) ---------------------------------------------- */
    
    /**
     * @brief sets NTP settings.
     * 
     * @param cfg 
     *    configruatino to be set
     * 
     * @throws std::runtime_error 
     *    if invalid @p cfg given
     */
    void set_ntp_config(const NtpConfig &cfg);

    /**
     * @brief Reads max time offset and delay.
     */
    NtpStatus get_ntp_status() const;
    
public: /* ------------------------------------ Public methods (configruation interface) ------------------------------------------ */

    /**
     * @brief Log into LMS1xx unit.
     * @details Increase privilege level, giving ability to change device configuration.
     */
    void login();

    /**
     * @returns
     *    current scan configruation
     */
    ScanConfig get_scan_config() const;

    /**
     * @brief Enumeration describing status of the @ref set_scan_config() action
     */
    enum class SetScanConfigStatus {
        Ok = 0,
        FrequencyError = 1,
        ResolutionError = 2,
        ResolutionAndScanAreaError = 3,
        ScanAreaError = 4,
        OtherError = 5
    };

    /**
     * @brief Sets scan configuration
     * 
     * @param cfg
     *    structure containing scan configuration
     */
    SetScanConfigStatus set_scan_config(const ScanConfig &cfg);

    /**
     * @brief Set scan data configuration.
     * @details Set format of scan message returned by device.
     * 
     * @param cfg
     *    structure containing scan data configuration.
     */
    void set_scan_data_config(const ScanDataConfig &cfg);

    /**
     * @brief Get current output range configuration
     * @returns scanOutputRange structure.
     */
    ScanOutputRange get_scan_output_range() const;

    /**
     * @brief Save current configuration permanently
     * @details Parameters are saved in the EEPROM of the LMS and will also be available after 
     *    the device is switched off and on again.
     */
    inline void save_current_config();

    /**
     * @brief The device is returned to the measurement mode after configuration
     */
    inline void logout();

public: /* ------------------------------------------ Public methods (utilities) -------------------------------------------------- */

    /// Converts @p status to human-readable string
    static constexpr std::string_view to_str(SetScanConfigStatus status);

protected: /* -------------------------------------------- Protected methods ------------------------------------------------------ */

    /**
     * @brief Creates I/O manager with he given @p log_context
     * 
     * @param log_context 
     *    (optional) logger context string to be used
     */
    inline lms1xx::IOManager make_io_manager(std::string_view log_context = "") const;

    /**
     * @brief Creates scan data parser
     */
    inline lms1xx::ScanDataParser make_scan_data_parser() const;
    
protected: /* ---------------------------------------------- Protected data ------------------------------------------------------- */

    // Logger interface (optional)
    mutable std::optional<rclcpp::Logger> logger;

    /// IP of the target LIDAR
    const std::string host;
    /// Socket descriptor for UDP connection (no value when driver not connected to the LIDAR)
    std::optional<int> socket_fd;

    /// Data buffer
    mutable lms1xx::ScanDataBuffer buffer;
};

/* ================================================================================================================================ */

} // End namespace sick

/* ==================================================== Implementation includes =================================================== */

// Auxiliary classes
#include "sick/lms1xx/scan_data_parser.hpp"
// Implementations
#include "sick/lms1xx/lms1xx.hpp"

/* ================================================================================================================================ */

#endif
