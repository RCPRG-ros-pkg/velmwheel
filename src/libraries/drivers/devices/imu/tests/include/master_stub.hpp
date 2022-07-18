/* ============================================================================================================================ *//**
 * @file       master_stub.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Tuesday, 21st June 2022 10:49:03 am
 * @modified   Sunday, 26th June 2022 12:01:58 pm
 * @project    engineering-thesis
 * @brief      Stub implementation of the ethercatlib Master interface
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __IMU_ETHERCAT_TEST_MASTER_STUB_H__
#define __IMU_ETHERCAT_TEST_MASTER_STUB_H__

/* =========================================================== Includes =========================================================== */

// Standard includes
#include <iostream>
// Private includes
#include "ethercat/master.hpp"
#include "slave_stub.hpp"

/* ============================================================= Class ============================================================ */

struct Master : public ethercat::Master<Master, Slave> {

    Master(const std::filesystem::path &eni_path) :
        ethercat::Master<Master, Slave>{ 
            eni_path,
            [](
                ::ethercat::eni::Slave slave_eni,
                std::vector<::ethercat::Slave<Slave>::Pdo<::ethercat::Slave<Slave>::PdoDirection::Input>> &&inputs,
                std::vector<::ethercat::Slave<Slave>::Pdo<::ethercat::Slave<Slave>::PdoDirection::Output>> &&outputs
            ) {
                return Slave{
                    slave_eni,
                    std::move(inputs),
                    std::move(outputs)
                };
            }
        },
        input_pdi_buffer (ethercat::eni::configruation_from_file(eni_path).get_process_image().get_size(ethercat::eni::ProcessImage::Direction::Inputs),  uint8_t(0)),
        output_pdi_buffer(ethercat::eni::configruation_from_file(eni_path).get_process_image().get_size(ethercat::eni::ProcessImage::Direction::Outputs), uint8_t(0))
    { }

    /// Sub implementation of get_state() method
    State get_state_impl([[maybe_unused]] std::chrono::milliseconds timeout) {
        return state;
    }

    /// Sub implementation of set_state() method
    void set_state_impl([[maybe_unused]] State state, [[maybe_unused]] std::chrono::milliseconds timeout) {
        state = state;
    }

    /// Sub implementation of read_bus() method
    void read_bus_impl([[maybe_unused]] ::ethercat::config::types::Span<uint8_t> pdi_buffer, [[maybe_unused]] std::chrono::milliseconds timeout) {
        std::copy(input_pdi_buffer.begin(), input_pdi_buffer.end(), pdi_buffer.begin());
    }

    /// Sub implementation of write_bus() method
    void write_bus_impl([[maybe_unused]] ::ethercat::config::types::Span<const uint8_t> pdi_buffer, [[maybe_unused]] std::chrono::milliseconds timeout) {
        std::copy(pdi_buffer.begin(), pdi_buffer.end(), output_pdi_buffer.begin());
    }

public:

    // Stub state holding target/source value of the set_state_impl()/get_state_impl() methods
    State state{ State::Init };

    // Stub buffer holding input PDI data
    std::vector<uint8_t> input_pdi_buffer;
    // Stub buffer holding output PDI data
    std::vector<uint8_t> output_pdi_buffer;

};

/* ================================================================================================================================ */

#endif
