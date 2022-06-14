/* ============================================================================================================================ *//**
 * @file       slave.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Monday, 14th February 2022 2:33:20 pm
 * @modified   Thursday, 28th April 2022 11:21:55 am
 * @project    engineering-thesis
 * @brief      Interface of the arbitrary EtherCAT Slave device associated with the cifx::Ethercat::Master object
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __CIFX_ETHERCAT_SLAVE_H__
#define __CIFX_ETHERCAT_SLAVE_H__

/* =========================================================== Includes =========================================================== */

// System includes
#include <memory>
#include <optional>
#include <functional>
// Private includes
#include "cifx/ethercat/common/pdo.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace cifx {
namespace ethercat {

/* ========================================================= Declarations ========================================================= */

// Forward declare Master class template
template<typename Lock>
class Master;

/**
 * @brief An abstract base class for EtherCAT slave's drivers cooperating with the
 *    @ref cifx::ethercat::Master class
 * 
 * @tparam Lock 
 *    type used to synchronsie classes' methods; has to implement lock() and unlock()
 *    methods
 */
class Slave : public std::enable_shared_from_this<Slave>  {

    // Make Master class friend o let it access registration and update methods
    template<typename Lock>
    friend class Master;

public: /* -------------------------------------------------- Public ctors & dtors ------------------------------------------------ */

    /**
     * @brief Construct a new Slave interface object
     */
    Slave() = default;

    /**
     * @brief Destroy the Slave object 
     */
    virtual ~Slave() = default;

private: /* -------------------------------------------- Private methods (registration) ------------------------------------------- */

    /**
     * @returns 
     *    description of PDOs registered by this Slave
     * 
     * @param eni_path
     *    path to the ENI file that the Master has been configured with
     */
    virtual pdo::DescriptorsSet get_pdos_description(std::string_view eni_path) = 0;

    /**
     * @brief This method is called from the @ref register_slave(...) method of the @ref Master object
     *    that the slave is registered to
     */
    virtual void on_registration() { };

    /**
     * @brief This method is called from the @ref unregister_slave(...) method of the @ref Master object
     *    that the slave is unregistered from
     */
    virtual void on_unregistration() { };

private: /* -------------------------------------------- Private methods (PDOs update) -------------------------------------------- */

    /**
     * @brief Provides updated value of the @p id'th input PDO to the slave
     * 
     * @param id 
     *    index of the updated input PDO
     * @param value 
     *    updated value of the PDO
     * 
     * @note This method is automatically called by the @ref Master for each slave's PDO that has been 
     *    registered after succesfull reading new data from the bus
     */
    virtual void update_input(std::size_t id, const pdo::Value &value) = 0;

    /**
     * @brief Updates @p id'th output PDO of the slave with the given @p value
     * 
     * @param id 
     *    index of the output PDO to be updated
     * @returns 
     *    current value for the PDO of ID @p id 
     * 
     * @throws std::out_of_range 
     *    if slave has no output PDO with the given @p id
     * @throws std::bad_variant_access 
     *    if type of the value passed in @p value does not correpsond to the preconfigured
     *    type of the PDO
     * 
     * @note This method is automatically called by the @ref Master for each slave's PDO that has been 
     *    registered before writting new data to the bus
     * @note This method is automatically called by the @ref Master for each slave's PDO that has been 
     *    registered when the slave is unregistered ( after calling @ref on_unregistration() )
     */
    virtual pdo::Value update_output(std::size_t id) = 0;
    
};

/* ================================================================================================================================ */

} // End namespace ethercat
} // End namespace cifx

#endif
