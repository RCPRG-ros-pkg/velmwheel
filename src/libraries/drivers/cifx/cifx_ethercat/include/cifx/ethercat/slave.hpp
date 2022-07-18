/* ============================================================================================================================ *//**
 * @file       slave.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 18th May 2022 9:11:53 am
 * @modified   Friday, 1st July 2022 1:22:41 pm
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
// CIFX includes
#include "EcmIF_Public.h"
// Private includes
#include "cifx.hpp"
#include "ethercat.hpp"

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
     * @brief Enume of possible states of the Slave in the ESM (EtherCAT State Machine)
     *    extended with CIFX-specific states of the interface device
     */
    enum class ExtendedState : std::underlying_type_t<State>{
        Init   = static_cast<std::underlying_type_t<State>>(State::Init),
        Preop  = static_cast<std::underlying_type_t<State>>(State::Preop),
        Boot   = static_cast<std::underlying_type_t<State>>(State::Boot),
        Safeop = static_cast<std::underlying_type_t<State>>(State::Safeop),
        Op     = static_cast<std::underlying_type_t<State>>(State::Op),
        Busoff,
        InitErr,
        PreopErr,
        BootErr,
        SafeopErr,
    };

    /**
     * @brief Structure describing current state of the slave
     */
    struct StateInfo {

        /// Current state
        ExtendedState current_state;
        /// Target state
        State target_state;

        /// If @c true tehe last stae change stopped with error
        bool error;

    };

public: /* ------------------------------------------------ Public static methods ------------------------------------------------- */

    /// Forward State-conversion function
    using SlaveInterfaceT::state_to_str;

    /**
     * @brief Converts @p state to the human-readable string
     * 
     * @param state 
     *    state to be converted
     * @returns 
     *    human-readable name of the state
     */
    static constexpr std::string_view state_to_str(ExtendedState state);

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
    StateInfo get_state_info(std::chrono::milliseconds timeout = std::chrono::milliseconds{ 1000 }) const;

public: /* --------------------------------------------- Public SDO-related methods ----------------------------------------------- */

    /**
     * @brief Enumeration describing mode in which @ref get_sdo_list() work
     * 
     */
    enum SDOListInfoMode {

        // Retrive list of all SDOs
        All = ECM_IF_COE_SDOINFO_GETODLIST_LIST_TYPE_ALL,
        // Retrive list of all RX-PDO-mappable SDOs
        RxPdioMappable = ECM_IF_COE_SDOINFO_GETODLIST_LIST_TYPE_RXPDOMAPPABLE,
        // Retrive list of all TX-PDO-mappable SDOs
        TxPdoMappable = ECM_IF_COE_SDOINFO_GETODLIST_LIST_TYPE_TXPDOMAPPABLE,
        // Retrive list of all SDOs necessary for backup
        Backup = ECM_IF_COE_SDOINFO_GETODLIST_LIST_TYPE_BACKUP,
        // Retrive list of all SDOs use dduring startup
        Settings = ECM_IF_COE_SDOINFO_GETODLIST_LIST_TYPE_SETTINGS
        
    };

    /**
     * @brief Reads list of indeces of SDOs provided by the slave
     * 
     * @param mode 
     *    filtering mode
     * @param timeout 
     *    access timeout
     * @returns 
     *    list of indeces of SDOs provided by the slave filtered according to the @p mode
     * 
     * @throws cifx::Error 
     *    on error
     */
    std::vector<uint16_t> get_sdo_list(
        SDOListInfoMode mode = SDOListInfoMode::All,
        std::chrono::milliseconds timeout = std::chrono::milliseconds{ 1000 }
    );

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
    State get_state_impl(std::chrono::milliseconds timeout) const;
    
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
     * @brief Type-independent implementation of the @ref upload_sdo(...) method template
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
    ) const;

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
