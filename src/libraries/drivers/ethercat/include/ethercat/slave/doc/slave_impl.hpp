/* ============================================================================================================================ *//**
 * @file       slave_impl.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 25th May 2022 7:37:33 pm
 * @modified   Monday, 13th June 2022 5:31:33 am
 * @project    engineering-thesis
 * @brief      Example interface of the class implementing abstract EtherCAT slave driver provided by the @a ethercat library
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifdef DOXYGEN_ONLY

/* =========================================================== Includes =========================================================== */

#include "ethercat/slave.hpp"

/* =========================================================== Namespace ========================================================== */

namespace ethercat::impl {

/* ======================================================== Implementation ======================================================== */

/**
 * @brief Example interface of the class implementing abstract EtherCAT slave driver 
 *    provided by the @a ethercat library
 */
class Slave : public ethercat::Slave<Slave> {

    /// Make base class a friend to let it access implementation methods
    friend class ethercat::Slave<Slave>;

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
     *    if invalid state identifier has been returned by the hardware
     */
    State get_state_impl(std::chrono::milliseconds timeout);
    
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
     * @tparam EntryRegistrationProxyT 
     *    type of the @p make_entry . @p EntryRegistrationProxyT is required to define a method template
     *    with signatre:
     * 
     *        template<abstract::pdo::Direction dir>
     *        abstract::pdo::Entry<dir> operator()(
     *            std::string_view pdo_name,
     *            const eni::Slave::Pdo::Entry &entry
     *        );
     * 
     *    that creates PDO entry for the requested object named @p object_name mapped into the
     *    PDO object named @p pdo_name .
     * 
     * @param slave_eni 
     *    parsing interface for the slave's description present in ENI file
     * @param process_image_eni 
     *    reference to the describtor of set of slave-related variables mapped into
     *    rhe Process Data Image
     * @param make_entry
     *    proxy object providing registration mechanism for slave's object entries mapped into
     *    the Process Data Image
     * 
     * @throws error 
     *    whatever @p make_entry throws
     */
    template<typename PdoEntryRegistrationProxyT>
    Slave(eni::Slave slave_eni, PdoEntryRegistrationProxyT make_entry);

    // Disable copy semantic
    Slave(const Slave &rslave) = delete;
    Slave &operator=(const Slave &rslave) = delete;
    // Enable moving semantic to let Slave be stored in realocatable buffer buffers
    Slave(Slave &&rslave) = default;
    Slave &operator=(Slave &&rslave) = default;

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
        ranges::span<const uint8_t> data,
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
        ranges::span<uint8_t> data,
        std::chrono::milliseconds timeout,
        bool complete_access
    );

};

/* ================================================================================================================================ */

} // End namespace ethercat::impl

#endif
