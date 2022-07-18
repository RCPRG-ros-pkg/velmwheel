/* ============================================================================================================================ *//**
 * @file       eni_parsing_test.cpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 18th May 2021 1:54:23 am
 * @modified   Saturday, 11th June 2022 5:46:57 pm
 * @project    engineering-thesis
 * @brief      Unit tests for the ENI (EtherCAT Network Informations) files parser.
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

/* =========================================================== Includes =========================================================== */

// System includes
#include <memory>
// Tetsing includes
#include "gtest/gtest.h"
// Private includes
#include "package_common/resources.hpp"
#include "ethercat/devices/imu.hpp"
#include "slave_stub.hpp"
#include "master_stub.hpp"

/* ======================================================= Common functions ======================================================= */

/**
 * @returns 
 *    path to the test ENI file
 */
auto get_test_eni_path() {

    // Name of the ament resource
    constexpr auto ament_resource_name = "test_data";
    // Name of the package
    constexpr auto package_name = "imu_ethercat";
    // Name of the test ENI file
    constexpr auto file_name = "test_eni.xml";

    // Get absolute path to the file
    return package_common::resources::get_file_path(package_name, ament_resource_name, file_name);
}

/// Log start string
constexpr auto log_start = "[    LOG   ] ";

/**
 * @returns 
 *    span over the PDI buffer of the master simulating actual value of the @p var_name CoE object
 *    of the @p slave_name slave
 */
ethercat::config::types::Span<uint8_t> get_slave_variable_span(
    Master &master,
    ethercat::eni::ProcessImage::Direction dir,
    auto slave_name,
    auto var_name
) {
    // Get ENI description of output PDO variables
    auto pdi_config = ethercat::eni::configruation_from_file(get_test_eni_path()).get_process_image();
    // Get ENI description of output PDO variable associated with the register of the slave
    auto variable_config = pdi_config.get_variables(dir).get_slave_variable(slave_name, var_name);

    // Get reference to the binary image of the register of the slave
    return ethercat::config::types::Span<uint8_t>(
        &((dir == ethercat::eni::ProcessImage::Direction::Outputs) ?
            master.output_pdi_buffer[variable_config->get_byte_offset()]:
            master.input_pdi_buffer[variable_config->get_byte_offset()]),
        variable_config->get_byte_size());
}

/* =========================================================== Fixtures =========================================================== */

using Imu = ethercat::devices::Imu<Slave>;


/// Tests suite state
class EthercatImuDriverTest : public testing::Test {
protected:

    // Skip setup of common resources
    static void SetUpTestSuite() { }
    // Skip teardown of common resources
    static void TearDownTestSuite() { }

    // Master driver
    static std::unique_ptr<Master> master;
    // Imu driver
    static std::unique_ptr<Imu> imu;

};

// Initializ master driver handle
std::unique_ptr<Master> EthercatImuDriverTest::master{ };
// Initializ motor driver handles
std::unique_ptr<Imu> EthercatImuDriverTest::imu{ };

/* ==================================================== Drivers initialization ==================================================== */

TEST_F(EthercatImuDriverTest, MasterConstruction) {

    // Print ENI path
    std::cout << "[    LOG   ] ENI file used: " << get_test_eni_path() << std::endl;

    // Parse ENI file
    try {  master = std::make_unique<Master>( get_test_eni_path() ); }
    // If thrown, report
    catch(std::exception &e) { FAIL() << e.what(); }
    
}


TEST_F(EthercatImuDriverTest, MotorDriversConstruction) {

    // Get <Master> tag parser
    try { imu = std::make_unique<Imu>(master->get_slave("Imu")); }
    // Fail on error
    catch(std::exception &e) { FAIL() << e.what(); }

}

/* ======================================================== Handlers tests ======================================================== */

TEST_F(EthercatImuDriverTest, InputHandlerTests) {

    using namespace std::literals::chrono_literals;

    bool flag = false;

    // Set input handler of the RL motor
    imu->set_measurement_read_handler([&flag](){ flag = true; });
    // Simulate input bus cycle
    master->read_bus(1s);
    // Assert flags values
    ASSERT_TRUE(flag);

}

/* ================================================================================================================================ */