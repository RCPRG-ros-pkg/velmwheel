/* ============================================================================================================================ *//**
 * @file       pdo.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 21st April 2022 2:57:03 am
 * @modified   Thursday, 5th May 2022 12:14:40 am
 * @project    engineering-thesis
 * @brief      Declarations of utilities used to describe PDOs
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __CIFX_ETHERCAT_COMMON_PDO_H__
#define __CIFX_ETHERCAT_COMMON_PDO_H__

/* =========================================================== Includes =========================================================== */

// System includes
#include <cstdint>
#include <variant>
#include <vector>
// Range includes
#include "range/v3/span.hpp"
// Private includes
#include "cifx/ethercat/common/utilities.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace cifx {
namespace ethercat {
namespace pdo {

/* ========================================================= Enumerations ========================================================= */

/**
 * @brief Enumeration of supported PDO data types
 */
enum class DataType : std::size_t {
    /* 8-bit boolean              */ Bool,
    /* 8-bit bitset               */ Byte,
    /* 16-bit bitset              */ Word,
    /* 32-bit bitset              */ DoubleWord,
    /* 8-bit signed int           */ ShortInt,
    /* 8-bit unsigned signed int  */ UnsignedShortInt,
    /* 16-bit signed int          */ Int,
    /* 16-bit unsigned signed int */ UnsignedInt,
    /* 32-bit signed int          */ DoubleInt,
    /* 32-bit unsigned signed int */ UnsignedDoubleInt,
    /* 64-bit signed int          */ LongInt,
    /* 64-bit unsigned signed int */ UnsignedLongInt,
    /* 32-bit floating point      */ Real,
    /* 64-bit floating point      */ LongReal
};

BIT      BOOL
BIT8     BOOL
BITARR8  BYTE
BITARR16 WORD
BITARR32 DWORD
INT8     SINT
INT16    INT
INT32    DINT
INT64    LINT
UINT8    USINT
UINT16   UINT
UINT32   UDINT
UINT64   ULINT
FLOAT    REAL
DOUBLE   LREAL

/* ========================================================== Data types ========================================================== */

/**
 * @brief Descriptor of the PDO
 */
struct Descriptor {

    // Offset of the PDO in the Process Data Image [in bytes]
    std::size_t offset;
    // Type of data
    DataType type;

};

/**
 * @brief Structure holding set of PDOs descriptors for a single Slave driver
 */
struct DescriptorsSet {

    // Input PDOs
    std::vector<Descriptor> in;
    // Output PDOs
    std::vector<Descriptor> out;

};

/**
 * @brief Structure refering to some PDO in the Process Data Image
 */
struct Reference {

    /// View of the reference PDO
    ranges::span<uint8_t> data;
    /// Type of data
    DataType type;
    
};

/**
 * @brief Structure holding complete set of PDOs for a single Slave driver
 */
struct ReferencesSet {

    // Input PDOs
    std::vector<Reference> in;
    // Output PDOs
    std::vector<Reference> out;

};

/**
 * @brief Union type used to store PDO value
 */
using Value = std::variant<
    /* DataType::Bool              */ bool,
    /* DataType::Byte              */ uint8_t,
    /* DataType::Word              */ uint16_t,
    /* DataType::DoubleWord        */ uint32_t,
    /* DataType::ShortInt          */ int8_t,
    /* DataType::UnsignedShortInt  */ uint8_t,
    /* DataType::Int               */ int16_t,
    /* DataType::UnsignedInt       */ uint16_t,
    /* DataType::DoubleInt         */ int32_t,
    /* DataType::UnsignedDoubleInt */ uint32_t,
    /* DataType::LongInt           */ int64_t,
    /* DataType::UnsignedLongInt   */ uint64_t,
    /* DataType::Real              */ float,
    /* DataType::LongReal          */ double
>;

/* ======================================================== Meta-functions ======================================================== */

/**
 * @brief Helper meta-function converting @ref DataType enumeration into corresponding data type
 */
template<DataType type>
using data_type_to_value_type = std::variant_alternative_t<to_underlying(type), Value>;

/**
 * @brief Helper meta-function converting @ref DataType enumeration into index of the corresponding 
 *    @ref Value alternative
 */
template<DataType type>
constexpr std::size_t data_type_to_index = to_underlying(type);

/* ======================================================== Free functions ======================================================== */

/**
 * @brief Makes @ref Value variant initialized with @p value on the value index
 *     corresponding to the @tparam type
 *  
 * @tparam type 
 *    type of the PDO to initialize @ref Value with
 * @param value 
 *    initial vale to be set
 * @returns 
 *    initialized @ref Value union
 */
template<DataType type>
inline Value make_value(data_type_to_value_type<type> value);

/**
 * @brief Sets @p pdo_value variant to @p value on the value index corresponding 
 *    to the @tparam type
 *  
 * @tparam type 
 *    type of the PDO to initialize @ref Value with
 * @param pdo_value 
 *    reference to @ref Value obejct to be set
 * @param value 
 *    initial vale to be set
 * @returns 
 *    initialized @ref Value union
 */
template<DataType type>
inline void set_value(Value &pdo_value, data_type_to_value_type<type> value);

/**
 * @param type 
 *    data type of the PDO
 * @returns 
 *    size of the data type corresponding to @p type in bytes
 */
std::size_t data_type_to_size(DataType type);

/**
 * @brief Resolves PDO reference to PDO value
 * 
 * @param reference 
 *    reference to be resolved
 * @returns 
 *    current value of the @p reference
 */
Value get_reference(const Reference &reference);

/**
 * @brief Sets PDO referenced by @p reference to @p value
 * 
 * @param reference 
 *    reference to be resolved
 * @returns 
 *    current value of the @p reference
 * 
 * @throws std::bad_variant_access 
 *    if type of the value passed in @p value does not correpsond to the 
 *    type of the @p reference
 */
void set_reference(Reference &reference, const Value &value);

/* ================================================================================================================================ */

} // End namespace pdo
} // End namespace ethercat
} // End namespace cifx

/* ==================================================== Implementation includes =================================================== */

#include "cifx/ethercat/common/pdo/pdo.hpp"

/* ================================================================================================================================ */

#endif
