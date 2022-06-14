/* ============================================================================================================================ *//**
 * @file       pdo.cpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Tuesday, 26th April 2022 3:40:21 pm
 * @modified   Thursday, 28th April 2022 11:27:52 am
 * @project    engineering-thesis
 * @brief      Definitions of utilities used to describe PDOs
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

/* =========================================================== Includes =========================================================== */

// System includes
#include <cstring>
// Private includes
#include "cifx/error.hpp"
#include "cifx/ethercat/common/pdo.hpp"
#include "cifx/ethercat/common/utilities.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace cifx {
namespace ethercat {
namespace pdo {

/* ======================================================= Helper functions ======================================================= */

/**
 * @brief Helper function template converting range of bytes into a value of type @tparam T emplaced
 *    in the @ref Value alternative
 * 
 * @tparam type
 *    target type
 * @param bytes 
 *    bytes to be converted
 * @returns 
 *    @p bytes converted to @tparam T value
 */
template<DataType type>
inline Value bytes_to_value(const ranges::span<uint8_t> bytes) {
    
    // Create the value
    data_type_to_value_type<type> value;
    // Copy raw bytes
    std::memcpy(
        static_cast<void*>(&value),
        static_cast<void*>(bytes.data()),
        bytes.size()
    );
    // Return index-constructed variant
    return make_value<type>(value);
}

/**
 * @brief Helper function template converting a PDO value corresponding to @tparam type 
 *    into a range of bytes 
 * 
 * @tparam type 
 *    source type
 * @param[in] value 
 *    source value
 * @param[out] bytes 
 *    target buffer
 */
template<DataType type>
inline void value_to_bytes(Value value, ranges::span<uint8_t> bytes) {
    
    // get value by type index
    data_type_to_value_type<type> &value_ref = std::get<data_type_to_index<type>>(value); 
    // Copy raw bytes
    std::memcpy(
        static_cast<void*>(bytes.data()),
        static_cast<void*>(&value_ref),
        bytes.size()
    );
}

/* ======================================================== Free functions ======================================================== */

std::size_t data_type_to_size(DataType type) {
    switch(type) {
        case DataType::Bool:              return sizeof(data_type_to_value_type<DataType::Bool>);
        case DataType::Byte:              return sizeof(data_type_to_value_type<DataType::Byte>);
        case DataType::Word:              return sizeof(data_type_to_value_type<DataType::Word>);
        case DataType::DoubleWord:        return sizeof(data_type_to_value_type<DataType::DoubleWord>);
        case DataType::ShortInt:          return sizeof(data_type_to_value_type<DataType::ShortInt>);
        case DataType::UnsignedShortInt:  return sizeof(data_type_to_value_type<DataType::UnsignedShortInt>);
        case DataType::Int:               return sizeof(data_type_to_value_type<DataType::Int>);
        case DataType::UnsignedInt:       return sizeof(data_type_to_value_type<DataType::UnsignedInt>);
        case DataType::DoubleInt:         return sizeof(data_type_to_value_type<DataType::DoubleInt>);
        case DataType::UnsignedDoubleInt: return sizeof(data_type_to_value_type<DataType::UnsignedDoubleInt>);
        case DataType::LongInt:           return sizeof(data_type_to_value_type<DataType::LongInt>);
        case DataType::UnsignedLongInt:   return sizeof(data_type_to_value_type<DataType::UnsignedLongInt>);
        case DataType::Real:              return sizeof(data_type_to_value_type<DataType::Real>);
        case DataType::LongReal:          return sizeof(data_type_to_value_type<DataType::LongReal>);
        default:
            throw cifx::Error{ CIFX_INVALID_PARAMETER, "[cifx::ethercat::pdo::get_reference] Invalid reference type" };
    }
}


Value get_reference(const Reference &reference) {
    switch(reference.type) {
        case DataType::Bool:              return bytes_to_value<DataType::Bool              >(reference.data);
        case DataType::Byte:              return bytes_to_value<DataType::Byte              >(reference.data);
        case DataType::Word:              return bytes_to_value<DataType::Word              >(reference.data);
        case DataType::DoubleWord:        return bytes_to_value<DataType::DoubleWord        >(reference.data);
        case DataType::ShortInt:          return bytes_to_value<DataType::ShortInt          >(reference.data);
        case DataType::UnsignedShortInt:  return bytes_to_value<DataType::UnsignedShortInt  >(reference.data);
        case DataType::Int:               return bytes_to_value<DataType::Int               >(reference.data);
        case DataType::UnsignedInt:       return bytes_to_value<DataType::UnsignedInt       >(reference.data);
        case DataType::DoubleInt:         return bytes_to_value<DataType::DoubleInt         >(reference.data);
        case DataType::UnsignedDoubleInt: return bytes_to_value<DataType::UnsignedDoubleInt >(reference.data);
        case DataType::LongInt:           return bytes_to_value<DataType::LongInt           >(reference.data);
        case DataType::UnsignedLongInt:   return bytes_to_value<DataType::UnsignedLongInt   >(reference.data);
        case DataType::Real:              return bytes_to_value<DataType::Real              >(reference.data);
        case DataType::LongReal:          return bytes_to_value<DataType::LongReal          >(reference.data);
        default:
            throw cifx::Error{ CIFX_INVALID_PARAMETER, "[cifx::ethercat::pdo::get_reference] Invalid reference type" };
    }
}


void set_reference(Reference &reference, const Value &value) {
    switch(reference.type) {
        case DataType::Bool:              value_to_bytes< DataType::Bool              >(value, reference.data); return;
        case DataType::Byte:              value_to_bytes< DataType::Byte              >(value, reference.data); return;
        case DataType::Word:              value_to_bytes< DataType::Word              >(value, reference.data); return;
        case DataType::DoubleWord:        value_to_bytes< DataType::DoubleWord        >(value, reference.data); return;
        case DataType::ShortInt:          value_to_bytes< DataType::ShortInt          >(value, reference.data); return;
        case DataType::UnsignedShortInt:  value_to_bytes< DataType::UnsignedShortInt  >(value, reference.data); return;
        case DataType::Int:               value_to_bytes< DataType::Int               >(value, reference.data); return;
        case DataType::UnsignedInt:       value_to_bytes< DataType::UnsignedInt       >(value, reference.data); return;
        case DataType::DoubleInt:         value_to_bytes< DataType::DoubleInt         >(value, reference.data); return;
        case DataType::UnsignedDoubleInt: value_to_bytes< DataType::UnsignedDoubleInt >(value, reference.data); return;
        case DataType::LongInt:           value_to_bytes< DataType::LongInt           >(value, reference.data); return;
        case DataType::UnsignedLongInt:   value_to_bytes< DataType::UnsignedLongInt   >(value, reference.data); return;
        case DataType::Real:              value_to_bytes< DataType::Real              >(value, reference.data); return;
        case DataType::LongReal:          value_to_bytes< DataType::LongReal          >(value, reference.data); return;
        default:
            throw cifx::Error{ CIFX_INVALID_PARAMETER, "[cifx::ethercat::pdo::set_reference] Invalid reference type" };
    }
}

/* ================================================================================================================================ */

} // End namespace pdo
} // End namespace ethercat
} // End namespace cifx

