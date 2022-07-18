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
#include "ethercat/devices/elmo.hpp"
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
    constexpr auto package_name = "elmo_ethercat";
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

using Elmo = ethercat::devices::Elmo<Slave>;

// Default configuration of the Elmo driver
constexpr auto DEFAULT_CONFIG = Elmo::Config{ };

/// Tests suite state
class EthercatElmoDriverTest : public testing::Test {
protected:

    // Skip setup of common resources
    static void SetUpTestSuite() { }
    // Skip teardown of common resources
    static void TearDownTestSuite() { }

    // Master driver
    static std::unique_ptr<Master> master;
    // Elmo drivers
    static std::unique_ptr<Elmo> rl_motor;
    static std::unique_ptr<Elmo> rr_motor;
    static std::unique_ptr<Elmo> fl_motor;
    static std::unique_ptr<Elmo> fr_motor;

};

// Initializ master driver handle
std::unique_ptr<Master> EthercatElmoDriverTest::master{ };
// Initializ motor driver handles
std::unique_ptr<Elmo> EthercatElmoDriverTest::rl_motor{ };
std::unique_ptr<Elmo> EthercatElmoDriverTest::rr_motor{ };
std::unique_ptr<Elmo> EthercatElmoDriverTest::fl_motor{ };
std::unique_ptr<Elmo> EthercatElmoDriverTest::fr_motor{ };

/* ==================================================== Drivers initialization ==================================================== */

TEST_F(EthercatElmoDriverTest, MasterConstruction) {

    // Print ENI path
    std::cout << "[    LOG   ] ENI file used: " << get_test_eni_path() << std::endl;

    // Parse ENI file
    try {  master = std::make_unique<Master>( get_test_eni_path() ); }
    // If thrown, report
    catch(std::exception &e) { FAIL() << e.what(); }
    
}


TEST_F(EthercatElmoDriverTest, MotorDriversConstruction) {

    // Get <Master> tag parser
    try { 
        rl_motor = std::make_unique<Elmo>(master->get_slave("WheelRearLeft"),   DEFAULT_CONFIG);
        rr_motor = std::make_unique<Elmo>(master->get_slave("WheelRearRight"),  DEFAULT_CONFIG);
        fl_motor = std::make_unique<Elmo>(master->get_slave("WheelFrontLeft"),  DEFAULT_CONFIG);
        fr_motor = std::make_unique<Elmo>(master->get_slave("WheelFrontRight"), DEFAULT_CONFIG);
    }
    // Fail on error
    catch(std::exception &e) { FAIL() << e.what(); }

}

/* ======================================================== Handlers tests ======================================================== */

TEST_F(EthercatElmoDriverTest, InputHandlerTests) {

    using namespace std::literals::chrono_literals;

    bool rl_motor_flag = false;
    bool rr_motor_flag = false;
    bool fl_motor_flag = false;
    bool fr_motor_flag = false;

    // Set input handler of the RL motor
    rl_motor->set_input_handler([&rl_motor_flag](){ rl_motor_flag = true; });
    // Simulate input bus cycle
    master->read_bus(1s);
    // Assert flags values
    ASSERT_TRUE(    rl_motor_flag);
    ASSERT_TRUE(not rr_motor_flag);
    ASSERT_TRUE(not fl_motor_flag);
    ASSERT_TRUE(not fr_motor_flag);

    // Set input handler of the RR motor
    rr_motor->set_input_handler([&rr_motor_flag](){ rr_motor_flag = true; });
    // Simulate input bus cycle
    master->read_bus(1s);
    // Assert flags values
    ASSERT_TRUE(    rl_motor_flag);
    ASSERT_TRUE(    rr_motor_flag);
    ASSERT_TRUE(not fl_motor_flag);
    ASSERT_TRUE(not fr_motor_flag);

    // Set input handler of the FL motor
    fl_motor->set_input_handler([&fl_motor_flag](){ fl_motor_flag = true; });
    // Simulate input bus cycle
    master->read_bus(1s);
    // Assert flags values
    ASSERT_TRUE(    rl_motor_flag);
    ASSERT_TRUE(    rr_motor_flag);
    ASSERT_TRUE(    fl_motor_flag);
    ASSERT_TRUE(not fr_motor_flag);

    // Set input handler of the FR motor
    fr_motor->set_input_handler([&fr_motor_flag](){ fr_motor_flag = true; });
    // Simulate input bus cycle
    master->read_bus(1s);
    // Assert flags values
    ASSERT_TRUE(    rl_motor_flag);
    ASSERT_TRUE(    rr_motor_flag);
    ASSERT_TRUE(    fl_motor_flag);
    ASSERT_TRUE(    fr_motor_flag);

}

TEST_F(EthercatElmoDriverTest, OutputHandlerTests) {

    using namespace std::literals::chrono_literals;

    bool rl_motor_flag = false;
    bool rr_motor_flag = false;
    bool fl_motor_flag = false;
    bool fr_motor_flag = false;

    // Set input handler of the RL motor
    rl_motor->set_output_handler([&rl_motor_flag](){ rl_motor_flag = true; });
    // Simulate output bus cycle
    master->write_bus(1s);
    // Assert flags values
    ASSERT_TRUE(    rl_motor_flag);
    ASSERT_TRUE(not rr_motor_flag);
    ASSERT_TRUE(not fl_motor_flag);
    ASSERT_TRUE(not fr_motor_flag);

    // Set input handler of the RR motor
    rr_motor->set_output_handler([&rr_motor_flag](){ rr_motor_flag = true; });
    // Simulate output bus cycle
    master->write_bus(1s);
    // Assert flags values
    ASSERT_TRUE(    rl_motor_flag);
    ASSERT_TRUE(    rr_motor_flag);
    ASSERT_TRUE(not fl_motor_flag);
    ASSERT_TRUE(not fr_motor_flag);

    // Set input handler of the FL motor
    fl_motor->set_output_handler([&fl_motor_flag](){ fl_motor_flag = true; });
    // Simulate output bus cycle
    master->write_bus(1s);
    // Assert flags values
    ASSERT_TRUE(    rl_motor_flag);
    ASSERT_TRUE(    rr_motor_flag);
    ASSERT_TRUE(    fl_motor_flag);
    ASSERT_TRUE(not fr_motor_flag);

    // Set input handler of the FR motor
    fr_motor->set_output_handler([&fr_motor_flag](){ fr_motor_flag = true; });
    // Simulate output bus cycle
    master->write_bus(1s);
    // Assert flags values
    ASSERT_TRUE(    rl_motor_flag);
    ASSERT_TRUE(    rr_motor_flag);
    ASSERT_TRUE(    fl_motor_flag);
    ASSERT_TRUE(    fr_motor_flag);

}

/* ==================================================== Driver examining tests ==================================================== */

TEST_F(EthercatElmoDriverTest, PdoMappingTest) {

    Elmo::PdoMappingInfo rl_mapping_info{ };

    // Test PDO mapping info getter of the RL motor
    ASSERT_NO_THROW(rl_mapping_info = rl_motor->get_pdo_mapping_info());
    // Test PDO mapping of the RL motor
    ASSERT_TRUE(     rl_mapping_info.has_digital_inputs  );
    ASSERT_TRUE(     rl_mapping_info.has_position        );
    ASSERT_TRUE(     rl_mapping_info.has_velocity        );
    ASSERT_TRUE( not rl_mapping_info.has_torque          );
    ASSERT_TRUE( not rl_mapping_info.has_current         );
    ASSERT_TRUE( not rl_mapping_info.has_target_position );
    ASSERT_TRUE(     rl_mapping_info.has_target_velocity );
    ASSERT_TRUE( not rl_mapping_info.has_target_torque   );

    Elmo::PdoMappingInfo rr_mapping_info{ };

    // Test PDO mapping info getter of the RR motor
    ASSERT_NO_THROW(rr_mapping_info = rr_motor->get_pdo_mapping_info());
    // Test PDO mapping of the RR motor
    ASSERT_TRUE(     rr_mapping_info.has_digital_inputs  );
    ASSERT_TRUE(     rr_mapping_info.has_position        );
    ASSERT_TRUE(     rr_mapping_info.has_velocity        );
    ASSERT_TRUE( not rr_mapping_info.has_torque          );
    ASSERT_TRUE( not rr_mapping_info.has_current         );
    ASSERT_TRUE( not rr_mapping_info.has_target_position );
    ASSERT_TRUE(     rr_mapping_info.has_target_velocity );
    ASSERT_TRUE( not rr_mapping_info.has_target_torque   );

    Elmo::PdoMappingInfo fl_mapping_info{ };

    // Test PDO mapping info getter of the FL motor
    ASSERT_NO_THROW(fl_mapping_info = fl_motor->get_pdo_mapping_info());
    // Test PDO mapping of the FL motor
    ASSERT_TRUE(     fl_mapping_info.has_digital_inputs  );
    ASSERT_TRUE(     fl_mapping_info.has_position        );
    ASSERT_TRUE(     fl_mapping_info.has_velocity        );
    ASSERT_TRUE( not fl_mapping_info.has_torque          );
    ASSERT_TRUE( not fl_mapping_info.has_current         );
    ASSERT_TRUE( not fl_mapping_info.has_target_position );
    ASSERT_TRUE(     fl_mapping_info.has_target_velocity );
    ASSERT_TRUE( not fl_mapping_info.has_target_torque   );

    Elmo::PdoMappingInfo fr_mapping_info{ };

    // Test PDO mapping info getter of the FR motor
    ASSERT_NO_THROW(fr_mapping_info = fr_motor->get_pdo_mapping_info());
    // Test PDO mapping of the FR motor
    ASSERT_TRUE(     fr_mapping_info.has_digital_inputs  );
    ASSERT_TRUE(     fr_mapping_info.has_position        );
    ASSERT_TRUE(     fr_mapping_info.has_velocity        );
    ASSERT_TRUE( not fr_mapping_info.has_torque          );
    ASSERT_TRUE( not fr_mapping_info.has_current         );
    ASSERT_TRUE( not fr_mapping_info.has_target_position );
    ASSERT_TRUE(     fr_mapping_info.has_target_velocity );
    ASSERT_TRUE( not fr_mapping_info.has_target_torque   );

}

/* ================================================== Device configuration tests ================================================== */

TEST_F(EthercatElmoDriverTest, ModeOfOperationReadTest) {

    using namespace ethercat::devices::elmo::registers;
    using namespace ethercat::devices::elmo::config;

    ModesOfOperation mode;

    // Get handle to the slave stub
    auto &slave = static_cast<Slave&>(master->get_slave("WheelRearLeft"));

    // Set target value of the mode
    slave.sdo.data = { static_cast<uint8_t>(ModesOfOperation::Velocity) };

    // Read mode of operation
    ASSERT_NO_THROW(mode = rl_motor->read_mode_of_operation());
    // Verify whether valid mode has been read
    ASSERT_EQ(slave.sdo.index,    MODES_OF_OPERATION.index  );
    ASSERT_EQ(slave.sdo.subindex, 0                         );
    ASSERT_EQ(mode,               ModesOfOperation::Velocity);
}

TEST_F(EthercatElmoDriverTest, ModeOfOperationWriteTest) {

    using namespace ethercat::devices::elmo::registers;
    using namespace ethercat::devices::elmo::config;

    // Get handle to the slave stub
    auto &slave = static_cast<Slave&>(master->get_slave("WheelRearLeft"));

    // Write mode of operation
    ASSERT_NO_THROW(rl_motor->write_mode_of_operation(ModesOfOperation::ProfiledVelocity));

    // Verify whether valid mode has been written
    ASSERT_EQ(slave.sdo.index,    MODES_OF_OPERATION.index                                               );
    ASSERT_EQ(slave.sdo.subindex, 0                                                                      );
    ASSERT_EQ(slave.sdo.data,     std::vector{ static_cast<uint8_t>(ModesOfOperation::ProfiledVelocity) });
}

/* ===================================================== Device controls tests ==================================================== */

TEST_F(EthercatElmoDriverTest, SetCommandTest) {
    
    using namespace ethercat::devices::elmo::registers;
    using namespace ethercat::devices::elmo::config;

    // Get reference to the binary image of the 'Control word' register of the slave
    ethercat::config::types::Span<uint8_t> cw_bin_image = 
        get_slave_variable_span(
            *master,
            ethercat::eni::ProcessImage::Direction::Outputs,
            "WheelRearLeft",
            "Control word"
        );

    // Prepare testing routine
    auto test_cw = [&cw_bin_image, this](Elmo::Command cmd) 
    {
        using namespace std::literals::chrono_literals;
        
        // Reset binary image of the PDO variable associated with the 'Control word' register of the slave
        std::fill((cw_bin_image).begin(), (cw_bin_image).end(), 0);
        
        // Write mode of operation
        try { rl_motor->set_command(cmd); }
        // If thrown, report
        catch(std::exception &e) { FAIL() << e.what(); } 
        
        // Simulate output bus cycle
        ASSERT_NO_THROW(master->write_bus(1s));
        
        // Prepare expected binary image of the register after output bus cycle (respect Little Endian order)
        std::vector<uint8_t> expected_cw_bin_image = {
            static_cast<uint8_t>(ethercat::common::utilities::to_underlying(cmd) &  0xFF),
            static_cast<uint8_t>(ethercat::common::utilities::to_underlying(cmd) >> 8   ),
        };
        
        // Verify whether valid binary image has been written
        ASSERT_EQ((cw_bin_image)[0], expected_cw_bin_image[0]);
        ASSERT_EQ((cw_bin_image)[1], expected_cw_bin_image[1]);
    };

    // Test cmd setting
    test_cw(Elmo::Command::Shutdown         );
    test_cw(Elmo::Command::SwitchOn         );
    test_cw(Elmo::Command::DisableVoltage   );
    test_cw(Elmo::Command::QuickStop        );
    test_cw(Elmo::Command::DisableOperation );
    test_cw(Elmo::Command::EnableOperation  );
    test_cw(Elmo::Command::PrepareFaultReset);
    test_cw(Elmo::Command::ResetFault       );
}

TEST_F(EthercatElmoDriverTest, GetStateTest) {
    
    using namespace ethercat::devices::elmo::registers;
    using namespace ethercat::devices::elmo::config;

    // Get reference to the binary image of the 'Status word' register of the slave
    ethercat::config::types::Span<uint8_t> sw_bin_image = 
        get_slave_variable_span(
            *master,
            ethercat::eni::ProcessImage::Direction::Inputs,
            "WheelRearLeft",
            "Status word"
        );

    /**
     * @brief Reserve some stack memory. Otherwise GTest will fail with "stack smashing detected"
     */
    {
        char buffer[1024];
        std::fill_n(buffer, 1024, 0);
    }

    // Prepare testing routine
    auto test_sw = [&sw_bin_image, this](Statusword state_rep, Elmo::State state)
    {
        using namespace std::literals::chrono_literals;
        
        // Prepare binary image of the register expected to be parsed as @p state
        sw_bin_image[0] = (state_rep.to_value() &  0xFF);
        sw_bin_image[1] = (state_rep.to_value() >> 8   );
        
        // Simulate input bus cycle
        ASSERT_NO_THROW(master->read_bus(1s));
        
        Elmo::State read_state;
        
        // Write mode of operation
        try { read_state = rl_motor->get_state(); } 
        // If thrown, report
        catch(std::exception &e) { FAIL() << e.what(); } 
        
        // Verify whether valid binary image has been written
        ASSERT_EQ(read_state, state);
    };
    
    // Test cmd setting
    test_sw(statusword::State::NotReadyToSwitchOn,  Elmo::State::NotReadyToSwitchOn );
    test_sw(statusword::State::SwitchedOnDisabled,  Elmo::State::SwitchedOnDisabled );
    test_sw(statusword::State::ReadyToSwitchOn,     Elmo::State::ReadyToSwitchOn    );
    test_sw(statusword::State::SwitchOn,            Elmo::State::SwitchOn           );
    test_sw(statusword::State::OperationEnabled,    Elmo::State::OperationEnabled   );
    test_sw(statusword::State::QuickStopActive,     Elmo::State::QuickStopActive    );
    test_sw(statusword::State::FaultReactionAcitve, Elmo::State::FaultReactionAcitve);
    test_sw(statusword::State::Fault,               Elmo::State::Fault              );

}

TEST_F(EthercatElmoDriverTest, GetDigitalInputsTest) {
    
    using namespace ethercat::devices::elmo::registers;
    using namespace ethercat::devices::elmo::config;

    // Get reference to the binary image of the 'Digital Inputs' register of the slave
    ethercat::config::types::Span<uint8_t> di_bin_image = 
        get_slave_variable_span(
            *master,
            ethercat::eni::ProcessImage::Direction::Inputs,
            "WheelRearLeft",
            "Digital Inputs"
        );

    /**
     * @brief Reserve some stack memory. Otherwise GTest will fail with "stack smashing detected"
     */
    {
        char buffer[1024];
        std::fill_n(buffer, 1024, 0);
    }

    // Prepare testing routine
    auto test_di = [&di_bin_image, this](DigitalInputs target_inputs)
    {
        using namespace std::literals::chrono_literals;
        
        // Prepare binary image of the register expected to be parsed as @p target_inputs
        di_bin_image[0] = ((target_inputs.to_value<uint32_t>() >> 0)  &  0xFF);
        di_bin_image[1] = ((target_inputs.to_value<uint32_t>() >> 8)  &  0xFF);
        di_bin_image[2] = ((target_inputs.to_value<uint32_t>() >> 16) &  0xFF);
        di_bin_image[3] = ((target_inputs.to_value<uint32_t>() >> 24) &  0xFF);
        
        // Simulate input bus cycle
        ASSERT_NO_THROW(master->read_bus(1s));
        
        DigitalInputs read_inputs;
        
        // Write mode of operation
        try { read_inputs = rl_motor->get_digital_inputs(); } 
        // If thrown, report
        catch(std::exception &e) { FAIL() << e.what(); } 
        
        // Verify whether valid binary image has been written
        ASSERT_EQ(read_inputs, target_inputs);
    };
    
    /// Use custom OR operator for enumerations
    using ethercat::utilities::operator|;

    // Test cmd setting
    test_di(DigitalInputs(DigitalInputs::IN1           | DigitalInputs::IN6                                ));
    test_di(DigitalInputs(DigitalInputs::Home                                                              ));
    test_di(DigitalInputs(DigitalInputs::Interlock     | DigitalInputs::IN3                                ));
    test_di(DigitalInputs(DigitalInputs::NegativeLimit | DigitalInputs::PositiveLimit | DigitalInputs::IN4 ));

}

/* ====================================================== Measurements tests ====================================================== */

constexpr double measurement_epsilon = 0.001;

/* -------------------------------------------------------------------------------------------------------------------------------- */

TEST_F(EthercatElmoDriverTest, PositionMeasurementTest) {
    
    using namespace ethercat::devices::elmo::registers;
    using namespace ethercat::devices::elmo::config;

    // Get reference to the binary image of the 'Position actual value' register of the slave
    ethercat::config::types::Span<uint8_t> pos_bin_image = 
        get_slave_variable_span(
            *master,
            ethercat::eni::ProcessImage::Direction::Inputs,
            "WheelRearLeft",
            "Position actual value"
        );

    /**
     * @brief Reserve some stack memory. Otherwise GTest will fail with "stack smashing detected"
     */
    {
        char buffer[1024];
        std::fill_n(buffer, 1024, 0);
    }

    // Prepare testing routine
    auto test_pos = [&pos_bin_image, this](double tested_gearing_shaft_pos_rad)
    {
        using namespace std::literals::chrono_literals;
        
        // Calculate target motor position (in encoder increments) manually
        int32_t target_pos_reg_value = static_cast<int32_t>(
            tested_gearing_shaft_pos_rad /
            (2.0 * M_PI / (DEFAULT_CONFIG.encoder_resolution * 4)) *
            DEFAULT_CONFIG.gear_ratio
        );

        /**
         * @note '* 4' part of the calculation refers to the quadruple multiplication of the
         *    counts frequency for quadruplet encoder wave
         */

        // Prepare binary image of the register expected to be parsed as @p target_inputs
        pos_bin_image[0] = ((target_pos_reg_value >> 0)  &  0xFF);
        pos_bin_image[1] = ((target_pos_reg_value >> 8)  &  0xFF);
        pos_bin_image[2] = ((target_pos_reg_value >> 16) &  0xFF);
        pos_bin_image[3] = ((target_pos_reg_value >> 24) &  0xFF);
        
        // Simulate input bus cycle
        ASSERT_NO_THROW(master->read_bus(1s));
        
        double position;
        
        // Write mode of operation
        try { position = rl_motor->get_position(); } 
        // If thrown, report
        catch(std::exception &e) { FAIL() << e.what(); }

        // Verify whether valid binary image has been written
        ASSERT_NEAR(position, tested_gearing_shaft_pos_rad, measurement_epsilon);
    };
    
    // Test various positions
    test_pos(  M_PI    );
    test_pos(  M_PI / 2);
    test_pos(  M_PI / 3);
    test_pos(  M_PI / 4);
    test_pos(  M_PI / 5);
    test_pos(  M_PI / 6);
    test_pos(- M_PI    );
    test_pos(- M_PI / 2);
    test_pos(- M_PI / 3);
    test_pos(- M_PI / 4);
    test_pos(- M_PI / 5);
    test_pos(- M_PI / 6);

}

TEST_F(EthercatElmoDriverTest, VelocityMeasurementTest) {
    
    using namespace ethercat::devices::elmo::registers;
    using namespace ethercat::devices::elmo::config;

    // Get reference to the binary image of the 'Velocity actual value' register of the slave
    ethercat::config::types::Span<uint8_t> vel_bin_image = 
        get_slave_variable_span(
            *master,
            ethercat::eni::ProcessImage::Direction::Inputs,
            "WheelRearLeft",
            "Velocity actual value"
        );

    /**
     * @brief Reserve some stack memory. Otherwise GTest will fail with "stack smashing detected"
     */
    {
        char buffer[1024];
        std::fill_n(buffer, 1024, 0);
    }

    // Prepare testing routine
    auto test_vel = [&vel_bin_image, this](double tested_gearing_shaft_speed_rad_s)
    {
        using namespace std::literals::chrono_literals;
        
        // Calculate target motor position (in encoder increments) manually
        int32_t target_speed_reg_value = static_cast<int32_t>(
            tested_gearing_shaft_speed_rad_s /
            (2.0 * M_PI / (DEFAULT_CONFIG.encoder_resolution * 4)) *
            DEFAULT_CONFIG.gear_ratio
        );

        /**
         * @note '* 4' part of the calculation refers to the quadruple multiplication of the
         *    counts frequency for quadruplet encoder wave
         */

        // Prepare binary image of the register expected to be parsed as @p target_inputs
        vel_bin_image[0] = ((target_speed_reg_value >> 0)  &  0xFF);
        vel_bin_image[1] = ((target_speed_reg_value >> 8)  &  0xFF);
        vel_bin_image[2] = ((target_speed_reg_value >> 16) &  0xFF);
        vel_bin_image[3] = ((target_speed_reg_value >> 24) &  0xFF);
        
        // Simulate input bus cycle
        ASSERT_NO_THROW(master->read_bus(1s));
        
        double velocity;
        
        // Write mode of operation
        try { velocity = rl_motor->get_velocity(); } 
        // If thrown, report
        catch(std::exception &e) { FAIL() << e.what(); }

        // Verify whether valid binary image has been written
        ASSERT_NEAR(velocity, tested_gearing_shaft_speed_rad_s, measurement_epsilon);
    };
    
    // Test various velocities
    test_vel(  M_PI    );
    test_vel(  M_PI / 2);
    test_vel(  M_PI / 3);
    test_vel(  M_PI / 4);
    test_vel(  M_PI / 5);
    test_vel(  M_PI / 6);
    test_vel(- M_PI    );
    test_vel(- M_PI / 2);
    test_vel(- M_PI / 3);
    test_vel(- M_PI / 4);
    test_vel(- M_PI / 5);
    test_vel(- M_PI / 6);

}

/* ======================================================== Setpoints tests ======================================================= */

constexpr double setpoint_epsilon = 0.001;

/* -------------------------------------------------------------------------------------------------------------------------------- */

TEST_F(EthercatElmoDriverTest, VelocitySetpointTest) {
    
    using namespace ethercat::devices::elmo::registers;
    using namespace ethercat::devices::elmo::config;

    // Get reference to the binary image of the 'Target Velocity' register of the slave
    ethercat::config::types::Span<uint8_t> vel_bin_image = 
        get_slave_variable_span(
            *master,
            ethercat::eni::ProcessImage::Direction::Outputs,
            "WheelRearLeft",
            "Target Velocity"
        );

    /**
     * @brief Reserve some stack memory. Otherwise GTest will fail with "stack smashing detected"
     */
    {
        char buffer[1024];
        std::fill_n(buffer, 1024, 0);
    }

    // Prepare testing routine
    auto test_vel = [&vel_bin_image, this](double tested_gearing_shaft_vel_rad_s)
    {
        using namespace std::literals::chrono_literals;
                
        // Write mode of operation
        try { rl_motor->set_velocity(tested_gearing_shaft_vel_rad_s); } 
        // If thrown, report
        catch(std::exception &e) { FAIL() << e.what(); }
        
        // Simulate output bus cycle
        ASSERT_NO_THROW(master->write_bus(1s));
        
        // Calculate expected motor velocity (in encoder increments per s) manually
        int32_t expected_vel_reg_value = static_cast<int32_t>(
            tested_gearing_shaft_vel_rad_s /
            (2.0 * M_PI / (DEFAULT_CONFIG.encoder_resolution * 4)) *
            DEFAULT_CONFIG.gear_ratio
        );

        /**
         * @note '* 4' part of the calculation refers to the quadruple multiplication of the
         *    counts frequency for quadruplet encoder wave
         */

        // Verify whether valid binary image has been written
        ASSERT_NEAR(vel_bin_image[0], static_cast<uint8_t>((expected_vel_reg_value >> 0)  &  0xFF), 128);
        ASSERT_NEAR(vel_bin_image[1], static_cast<uint8_t>((expected_vel_reg_value >> 8)  &  0xFF),   0);
        ASSERT_NEAR(vel_bin_image[2], static_cast<uint8_t>((expected_vel_reg_value >> 16) &  0xFF),   0);
        ASSERT_NEAR(vel_bin_image[3], static_cast<uint8_t>((expected_vel_reg_value >> 24) &  0xFF),   0);

        /**
         * @note Expectation toleration value has been set to @c 128 encoder increments per second.
         *   For the default configuration of the Elmo driver, i.e.:
         * 
         *      - resolution: 2'500 pprev (times 4 due to quadruple wave format)
         *      - gear ratio: 50
         * 
         *   results in gearing's shaft speed tolerance smaller than ~0.0016 [rad/s] (~0.092 [deg/s])
         */

    };
    
    // Test various positions
    test_vel(  M_PI    );
    test_vel(  M_PI / 2);
    test_vel(  M_PI / 3);
    test_vel(  M_PI / 4);
    test_vel(  M_PI / 5);
    test_vel(  M_PI / 6);
    test_vel(- M_PI    );
    test_vel(- M_PI / 2);
    test_vel(- M_PI / 3);
    test_vel(- M_PI / 4);
    test_vel(- M_PI / 5);
    test_vel(- M_PI / 6);

}

/* ================================================================================================================================ */