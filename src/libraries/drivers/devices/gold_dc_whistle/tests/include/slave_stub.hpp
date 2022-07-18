/* ============================================================================================================================ *//**
 * @file       slave_stub.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Tuesday, 21st June 2022 10:49:03 am
 * @modified   Sunday, 26th June 2022 12:01:58 pm
 * @project    engineering-thesis
 * @brief      Stub implementation of the ethercatlib Slave interface
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __ELMO_ETHERCAT_TEST_SLAVE_STUB_H__
#define __ELMO_ETHERCAT_TEST_SLAVE_STUB_H__

/* =========================================================== Includes =========================================================== */

// Standard includes
#include <iostream>
// Private includes
#include "ethercat/slave.hpp"

/* ============================================================= Class ============================================================ */

struct Slave : public ethercat::Slave<Slave> {
public:

    Slave(
        ::ethercat::eni::Slave slave_eni,
        std::vector<::ethercat::Slave<Slave>::Pdo<::ethercat::Slave<Slave>::PdoDirection::Input>> &&inputs,
        std::vector<::ethercat::Slave<Slave>::Pdo<::ethercat::Slave<Slave>::PdoDirection::Output>> &&outputs
    ) : 
        ethercat::Slave<Slave> { slave_eni, std::move(inputs), std::move(outputs) }
    { }

public:

    /// Stub implementation of get_state() method
    State get_state_impl([[maybe_unused]] std::chrono::milliseconds timeout) {
        return state;
    }

    /// Stub implementation of set_state() method
    void set_state_impl([[maybe_unused]] State state, [[maybe_unused]] std::chrono::milliseconds timeout) {
        state = state;
    }

    /// Stub implementation of download_sdo() method
    void download_sdo(
        [[maybe_unused]] uint16_t index,
        [[maybe_unused]] uint16_t subindex,
        [[maybe_unused]] ::ethercat::config::types::Span<const uint8_t> data,
        [[maybe_unused]] std::chrono::milliseconds timeout,
        [[maybe_unused]] bool complete_access
    ) {
        // Copy data
        sdo.data.resize(data.size());
        std::copy(data.begin(), data.end(), sdo.data.begin());
        // Keep index info
        sdo.index    = index;
        sdo.subindex = subindex;
    }

    /// Stub implementation of upload_sdo() method
    void upload_sdo(
        [[maybe_unused]] uint16_t index,
        [[maybe_unused]] uint16_t subindex,
        [[maybe_unused]] ::ethercat::config::types::Span<uint8_t> data,
        [[maybe_unused]] std::chrono::milliseconds timeout,
        [[maybe_unused]] bool complete_access
    ) {
        // Copy data
        assert(sdo.data.size() == static_cast<std::size_t>(data.size()));
        std::copy(sdo.data.begin(), sdo.data.end(), data.begin());
        // Keep index info
        sdo.index    = index;
        sdo.subindex = subindex;
    }

public:

    // Stub state holding target/source value of the set_state_impl()/get_state_impl() methods
    State state { State::Init };

    // Stub buffer holding target/source values of the download_sdo()/upload_sdo() methods
    struct {
        uint16_t index { 0 };
        uint16_t subindex { 0 };
        std::vector<uint8_t> data;
    } sdo;

};

/* ================================================================================================================================ */

#endif
