/* ============================================================================================================================ *//**
 * @file       slave.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 18th May 2022 9:11:53 am
 * @modified   Monday, 13th June 2022 11:31:09 am
 * @project    engineering-thesis
 * @brief      Definition of the Slave class representing slave device on the EtherCAT bus
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __CIFX_ETHERCAT_SLAVE_H__
#define __CIFX_ETHERCAT_SLAVE_H__

/* =========================================================== Includes =========================================================== */

// Standard includes
#include <chrono>
// Private includes
#include "cifx.hpp"
#include "ethercat/slave.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace cifx::ethercat {

/* ========================================================== Data types ========================================================== */

// Forward declare Master class 
class Master;

/**
 * @brief Implementation of the @ref ::ethercat::Slave driver with the CIFX/netX
 *   C++ Toolkit hardware access layer
 */
class Slave : public ::ethercat::Slave<Slave> {

    /// Make Master class a friend to let it access slave's constructor
    friend class Master;
    /// Make base class a friend to let it access implementation methods
    friend class ::ethercat::Slave<Slave>;

public: /* ---------------------------------------------------- Public types ------------------------------------------------------ */

    /// Type of the underlying Slave interface
    using SlaveInterfaceT = ::ethercat::Slave<Slave>;

    /**
     * @brief Structure describing current state of the slave
     */
    struct StateInfo {

        /// Current state
        State current_state;
        /// Target state
        State target_state;

        /// If @c true tehe last stae change stopped with error
        bool error;

    };

public: /* ----------------------------------------------------- Public ctors ----------------------------------------------------- */

    /// Default moving semantic
    Slave(Slave &&rslave) = default;
    Slave &operator=(Slave &&rslave) = default;

public: /* ------------------------------------------------ Public common methods ------------------------------------------------- */

    /**
     * @brief Reads current state of the slave device in the ESM (EtherCAT slave machine)
     *   along with CIFX-specific informations
     * 
     * @param timeout 
     *    access timeout
     * @returns 
     *    current state of the slave device in the ESM (EtherCAT slave machine)
     * 
     * @throws cifx::Error 
     *    on error
     * @throws std::range_error
     *    if invalid state identifier has been returned by the CIFX device
     */
    StateInfo get_state_info(std::chrono::milliseconds timeout = std::chrono::milliseconds{ 100 });

protected: /* ------------------------------ Protected EtherCAT common methods (implementations) ---------------------------------- */

    /**
     * @brief Reads current state of the slave device in the ESM (EtherCAT slave machine)
     * 
     * @param timeout 
     *    access timeout
     * @returns 
     *    current state of the slave device in the ESM (EtherCAT slave machine)
     * 
     * @throws cifx::Error 
     *    on error
     * @throws std::range_error
     *    if invalid state identifier has been returned by the CIFX device
     */
    inline State get_state_impl(std::chrono::milliseconds timeout);
    
    /**
     * @brief Requestes state change of the slave device in the ESM (EtherCAT slave machine)
     * 
     * @param state 
     *    target state
     * @param timeout 
     *    access timeout
     * 
     * @throws cifx::Error 
     *    on error
     * @throws std::range_error
     *    if invalid state has been requested
     */
    void set_state_impl(State state, std::chrono::milliseconds timeout);

protected: /* --------------------------------------------- Protected ctors & dtors ----------------------------------------------- */

    /**
     * @brief Construct a new Slave interface
     * 
     * @param channel
     *    reference tot he CIFX channel used to communicate with CIFX master device
     * @param slave_eni 
     *    parsing interface for the slave's description present in ENI file
     * @param inputs 
     *    input PDOs associated with the slave
     * @param outputs 
     *    output PDOs associated with the slave
     * 
     * @throws cifx::ethercat::eni::Error
     *    if initialization fails for some reason
     */
    inline Slave(
        cifx::Channel &channel,
        ::ethercat::eni::Slave slave_eni,
        std::vector<Pdo<PdoDirection::Input>> &&inputs,
        std::vector<Pdo<PdoDirection::Output>> &&outputs
    );

    /// Disable copy semantic
    Slave(const Slave &rslave) = delete;
    Slave &operator=(const Slave &rslave) = delete;

protected: /* -------------------------------------- Protected implementation methods (SDO) --------------------------------------- */

    /**
     * @brief Type-independent implementation of the @ref download_sdo(...) method template
     * 
     * @param index 
     *    index of the object
     * @param subindex 
     *    subindex of the object
     * @param data 
     *    data to be written into the object
     * @param timeout 
     *    access timeout
     * @param complete_access 
     *    @c true if complete access is to be performed
     */
    void download_sdo(
        uint16_t index,
        uint16_t subindex,
        ::ethercat::config::types::Span<const uint8_t> data,
        std::chrono::milliseconds timeout,
        bool complete_access
    );
    
    /**
     * @brief Type-independent implementation of the @ref upload_object(...) method template
     * 
     * @param index 
     *    index of the object
     * @param subindex 
     *    subindex of the object
     * @param[inout] data 
     *    data to be written into the object
     * @param timeout 
     *    access timeout
     * @param complete_access 
     *    @c true if complete access is to be performed
     */
    void upload_sdo(
        uint16_t index,
        uint16_t subindex,
        ::ethercat::config::types::Span<uint8_t> data,
        std::chrono::milliseconds timeout,
        bool complete_access
    );

private: /* --------------------------------------------------- Private data ------------------------------------------------------ */

    /// Reference tot he CIFX channel used to communicate with CIFX master device
    cifx::Channel &channel;

};

/* ================================================================================================================================ */

} // End namespace cifx::ethercat

/* ==================================================== Implementation includes =================================================== */

#include "cifx/ethercat/slave/slave.hpp"

/* ================================================================================================================================ */

#endif
