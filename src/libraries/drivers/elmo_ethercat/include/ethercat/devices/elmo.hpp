/* ============================================================================================================================ *//**
 * @file       elmo.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 28th April 2022 9:24:14 pm
 * @modified   Monday, 13th June 2022 8:14:26 pm
 * @project    engineering-thesis
 * @brief      Definition of the EtherCAT driver implementation 
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __ETHERCAT_DEVICES_ELMO_H__
#define __ETHERCAT_DEVICES_ELMO_H__

/* =========================================================== Includes =========================================================== */

// Standard includes
#include <cmath>
// Private includes
#include "ethercat/slave.hpp"
#include "ethercat/devices/elmo/registers.hpp"
#include "ethercat/devices/elmo/config.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace ethercat::devices {

/* ============================================================= Class ============================================================ */

/**
 * @brief Basic driver of the Elmo srvomoto driver based on the :ethercat:`ethercat<>` library
 * @details At the moment the driver support only basic functionalities of the servodriver
 *    including manipulation of the controlword/statusword CoE objects, providing target values
 *    for position & velocity and reading current values of position/velocity/torque/current.
 *    Otheri functionalities has been not implemented yet [1]
 * 
 * @tparam SlaveImplementationT 
 *    type of the object implementing hardware-dependent elements of the Slave interface
 * 
 * @note [1] 'Functionalities has not been implemented yet' should be understood in kind of similiar 
 *    way as 'Japenees have not produced nuclear weapon yet'.
 */
template<typename SlaveImplementationT>
class Elmo {

public: /* --------------------------------------------------- Public types ------------------------------------------------------ */

    // Type of the underlying driver
    using SlaveInterface = Slave<SlaveImplementationT>;

    /// Multiplication factor of quadrature encoder's pulse frequency ( @ref Config::encoder_resolution )
    static constexpr std::size_t QuadratureFormatPulseMultiplicationFactor = 4;

    /**
     * @brief Configuration structure of the driver
     */
    struct Config {

        /**
         * @brief Number of pulses produced by the encoder per revolution of the motor shaft
         * @details This value shoulld indicate number of <b>quadrature cycles</b> output by
         *    the encoder per revolution of the motor shaft. 
         * 
         * @note At each quadrature cycle of the encoder servodriver counts 4-micro-steps  
         *   (and so it is called 'quadrature' cycle). In result, the effective resolution
         *   of the encoder is 4-times higher. E.g. encoders currently mounted in the WUT
         *   Velmwheel robot produce 2500ppr (pulses per rev) and it is the value that should
         *   be given in the @a encoder_resolution field . However, the servodriver will count
         *   10'000 'ticks' per revolution of the motor's shaft. This scaling factor is
         *   taken into account by the driver.
         */
        uint32_t encoder_resolution { 2'500 };

        /**
         * @brief Gear ratio of the gearing coupled with the servomotor
         */
        double gear_ratio { 1 };

        /**
         * @brief Rated torque of the motor in [Nm]
         * 
         * @note Default value is given for PSM60 servomotor present in the WUT Velmwheel robot
         */
        double motor_rate_torque{ 1.27 };

        /**
         * @brief Rated torque of the motor in [A]
         * 
         * @note Default value is given for PSM60 servomotor present in the WUT Velmwheel robot
         */
        double motor_rate_current{ 11 };

        /**
         * @todo Extend Config structure with other parameters that could be set to non-default
         *    values at driver's construction. At the moment, the minimal driving scheme is 
         *    required and so only encoder-scaling factor is required for proper functioning.
         */

    };

    /**
     * @brief Commands that can be given to the servodriver via "Controlword" object
     */
    enum class Command : uint16_t { 
        Shutdown          = elmo::config::controlword::State::Shutdown.to_value(),
        SwitchOn          = elmo::config::controlword::State::SwitchOn.to_value(),
        DisableVoltage    = elmo::config::controlword::State::DisableVoltage.to_value(),
        QuickStop         = elmo::config::controlword::State::QuickStop.to_value(),
        DisableOperation  = elmo::config::controlword::State::DisableOperation.to_value(),
        EnableOperation   = elmo::config::controlword::State::EnableOperation.to_value(),
        PrepareFaultReset = elmo::config::controlword::State::BeforeFaultReset.to_value(),
        ResetFault        = elmo::config::controlword::State::AfterFaultReset.to_value()
    } ;

    /**
     * @brief States of the device obtained via "Statusword" object
     */
    enum class State { 
        NotReadyToSwitchOn,
        SwitchedOnDisabled,
        ReadyToSwitchOn,
        SwitchOn,
        OperationEnabled,
        QuickStopActive,
        FaultReactionAcitve,
        Fault
    };

public: /* ------------------------------------------------- Public ctors & dtors ------------------------------------------------ */

    /**
     * @brief Constructs the driver
     * @param slave 
     *    handle to the device driver interface
     * @param config 
     *    configuration of the driver
     */
    inline Elmo(SlaveInterface &slave, const Config &config);

    /**
     * @brief Destroy the Elmo object cleaning up the driver interface
     */
    inline ~Elmo();

    /// Disable copy-semantics
    Elmo(const Elmo &rimu) = delete;
    Elmo &operator=(const Elmo &rimu) = delete;
    /// Default move-semantics
    Elmo(Elmo &&rimu) = default;
    Elmo &operator=(Elmo &&rimu) = default;

public: /* -------------------------------------------- Public configuration methods --------------------------------------------- */

    /**
     * @brief Sets handler routine called when new Elmo state & measurements are read from the bus
     * 
     * @tparam HandlerT 
     *    handler type
     * @param handler 
     *    handler to be set
     */
    template<typename HandlerT>
    inline void set_input_handler(HandlerT &&handler);

    /**
     * @brief Sets handler routine called when cyclical data need to be updated before being pushed to the bus
     * 
     * @tparam HandlerT 
     *    handler type
     * @param handler 
     *    handler to be set
     */
    template<typename HandlerT>
    inline void set_output_handler(HandlerT &&handler);

public: /* -------------------------------------------- Public examining methods ------------------------------------------------- */

    struct PdoMappingInfo {

        /// @c true if device has it's "Digital inputs" entry mapped in the Process Data Image buffer
        bool has_digital_inputs;
        
        /// @c true if device has it's "Position actual value" entry mapped in the Process Data Image buffer
        bool has_position;
        /// @c true if device has it's "Velocity actual value" entry mapped in the Process Data Image buffer
        bool has_velocity;
        /// @c true if device has it's "Torque actual value" entry mapped in the Process Data Image buffer
        bool has_torque;
        /// @c true if device has it's "Current actual value" entry mapped in the Process Data Image buffer
        bool has_current;

        /// @c true if device has it's "Target position" entry mapped in the Process Data Image buffer
        bool has_target_position;
        /// @c true if device has it's "Target velocity" entry mapped in the Process Data Image buffer
        bool has_target_velocity;
        /// @c true if device has it's "Target torque" entry mapped in the Process Data Image buffer
        bool has_target_torque;   
        
    };

    /**
     * @returns 
     *    structure describing which of optional PDO entries of the slave are actually
     *    mapped into the Process Data Image
     */
    PdoMappingInfo get_pdo_mapping_info() const;

public: /* ---------------------------------------- Public device configuration methods ------------------------------------------ */

    /**
     * @brief Reads "Modes of operation" object from the device
     */
    inline elmo::config::ModesOfOperation read_mode_of_operation() const;

    /**
     * @brief Writes a new value of the "Modes of operation" object to the the device
     */
    inline void read_mode_of_operation(elmo::config::ModesOfOperation mode);

public: /* ------------------------------------------- Public device-control methods --------------------------------------------- */

    /**
     * @brief Sets value of the "Controlword" object that will be written to the device
     *    on the next bus cycle
     * @param command
     *    command to be set
     */
    void set_command(Command command);

    /**
     * @brief Reads value of the "Controlword" object that will be written to the device
     *    on the next bus cycle
     */
    Command get_command() const;

    /**
     * @brief Reads last state of the device received via PDO communication of 
     *    "Statusword" object
     * 
     * @throws std::out_of_range 
     *    if invalid statusword has been received
     */
    State get_state() const;

    /**
     * @brief Reads last state of digital inputs received via PDO communication
     * 
     * @throws std::out_of_range 
     *    if slave device has not been configured with digital inputs mapped into
     *    Process Data Image
     */
    elmo::config::DigitalInputs get_digital_inputs() const;

public: /* ---------------------------------------- Public measurements-related methods ------------------------------------------ */
    
    /**
     * @brief Reads last value of the motor's position received via PDO communication
     * 
     * @returns 
     *    position of the gearing's shaft in [rad] (driver automatically calculates
     *    position of gearing's shaft based on the resolution of encoder and gear ratio
     *    given in the constructor)
     * 
     * @throws std::out_of_range 
     *    if slave device has not been configured with position reading mapped into
     *    Process Data Image
     */
    double get_position() const;
    
    /**
     * @brief Reads last value of the motor's velocity received via PDO communication
     * 
     * @returns 
     *    velocity of the gearing's shaft in [rad/s] (driver automatically calculates
     *    velocity of gearing's shaft based on the resolution of encoder and gear ratio
     *    given in the constructor)
     * 
     * @throws std::out_of_range 
     *    if slave device has not been configured with velocity reading mapped into
     *    Process Data Image
     */
    double get_velocity() const;
    
    /**
     * @brief Reads last value of the motor's torque received via PDO communication
     * 
     * @returns 
     *    torque applied by teh gearing's shaft in [Nm] (driver automatically calculates
     *    torque applied by teh gearing's shaft based on the gear ratio given in the 
     *    constructor)
     * 
     * @throws std::out_of_range 
     *    if slave device has not been configured with torque reading mapped into
     *    Process Data Image
     */
    double get_torque() const;
    
    /**
     * @brief Reads last value of the motor's current received via PDO communication
     * 
     * @returns 
     *    amount of current running throught the motor's winding in [A]
     * 
     * @throws std::out_of_range 
     *    if slave device has not been configured with current reading mapped into
     *    Process Data Image
     */
    double get_current() const;

public: /* ------------------------------------------ Public setpoints-related methods ------------------------------------------- */
    
    /**
     * @brief Sets target position of the gearing's shaft to be sent to the driver on th next
     *   bus cycle via PDO communication
     * 
     * @param position
     *    target position of the gearing's shaft in [rad]
     * 
     * @throws std::out_of_range 
     *    if slave device has not been configured with position target mapped into
     *    Process Data Image
     */
    void set_position(double position);
    
    /**
     * @brief Sets target velocity of the gearing's shaft to be sent to the driver on th next
     *   bus cycle via PDO communication
     * 
     * @param velocity
     *    target velocity of the gearing's shaft in [rad/s]
     * 
     * @throws std::out_of_range 
     *    if slave device has not been configured with velocity target mapped into
     *    Process Data Image
     */
    void set_velocity(double velocity);
    
    /**
     * @brief Sets target torque applied by the gearing's shaft to be sent to the driver on th next
     *   bus cycle via PDO communication
     * 
     * @param torque
     *    target torque applied by teh gearing's shaft in [Nm]
     * 
     * @throws std::out_of_range 
     *    if slave device has not been configured with torque target mapped into
     *    Process Data Image
     */
    void set_torque(double torque);

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

    /// Mechanical configuration of the motor
    Config config;

private: /* ---------------------------------------------- Private data (SDO) ---------------------------------------------------- */

    /// SDO interface used to set/read current mode of operation
    Sdo<elmo::config::ModesOfOperation> mode_of_operation_sdo;

private: /* ---------------------------------------------- Private data (PDO) ---------------------------------------------------- */

    /// Reference to the "Statusword" entry in the Process Data Image buffer
    InputPdoEntry<uint16_t> statusword_pdo;
    /// Reference to the "Controlword" entry in the Process Data Image buffer
    OutputPdoEntry<Command> controlword_pdo;
    
    /// Reference to the "Digital inputs" entry in the Process Data Image buffer
    std::optional<InputPdoEntry<uint32_t>> digital_inputs_pdo;
    
    /// Reference to the "Position actual value" entry in the Process Data Image buffer
    std::optional<InputPdoEntry<int32_t>> position_pdo;
    /// Reference to the "Velocity actual value" entry in the Process Data Image buffer
    std::optional<InputPdoEntry<int32_t>> velocity_pdo;
    /// Reference to the "Torque actual value" entry in the Process Data Image buffer
    std::optional<InputPdoEntry<int16_t>> torque_pdo;
    /// Reference to the "Current actual value" entry in the Process Data Image buffer
    std::optional<InputPdoEntry<int16_t>> current_pdo;

    /// Reference to the "Target position" entry in the Process Data Image buffer
    std::optional<OutputPdoEntry<int32_t>> target_position_pdo;
    /// Reference to the "Target velocity" entry in the Process Data Image buffer
    std::optional<OutputPdoEntry<int32_t>> target_velocity_pdo;
    /// Reference to the "Target torque" entry in the Process Data Image buffer
    std::optional<OutputPdoEntry<int16_t>> target_torque_pdo;   

};

/* ================================================================================================================================ */

} // End namespace ethercat::devices

/* ==================================================== Implementation includes =================================================== */

#include "ethercat/devices/elmo/elmo.hpp"

/* ================================================================================================================================ */

#endif
