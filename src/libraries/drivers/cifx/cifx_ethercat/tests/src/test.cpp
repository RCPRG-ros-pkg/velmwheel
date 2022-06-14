/* ============================================================================================================================ *//**
 * @file       test.cpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 18th May 2021 1:54:23 am
 * @modified   Friday, 27th May 2022 2:19:08 am
 * @project    engineering-thesis
 * @brief      Unit tests for the EtherCAT driver. Test is desire to run on machine with CIFX card up and running.
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

/* =========================================================== Includes =========================================================== */

// System includes
#include <memory>
#include <thread>
// Tetsing includes
#include "gtest/gtest.h"
// Private includes
#include "cifx/utilities.hpp"
#include "cifx/ethercat.hpp"
#include "package_common/resources.hpp"

/* ======================================================= Common functions ======================================================= */

/**
 * @returns 
 *    path to the test ENI file
 */
auto get_test_eni_path() {

    // Name of the ament resource
    constexpr auto ament_resource_name = "test_data";
    // Name of the package
    constexpr auto package_name = "cifx_ethercat";
    // Name of the test ENI file
    constexpr auto file_name = "test_eni.xml";

    // Get absolute path to the file
    return package_common::resources::get_file_path(package_name, ament_resource_name, file_name);
}

/// Log start string
constexpr auto log_start = "[    LOG   ] ";

/* =========================================================== Fixtures =========================================================== */

class CifxEthercatTest : public testing::Test {
protected:

    // Skip setup of common resources
    static void SetUpTestSuite() { }
    // Skip teardown of common resources
    static void TearDownTestSuite() { }

    // Path to the bootloader file
    static std::string bootloader_path;
    // Path to the firmware file
    static std::string firmware_path;
    // Path to the config file (ENI)
    static std::string config_path;

    // Cifx Toolkit driver
    static std::unique_ptr<cifx::Driver> driver;
    // Cifx device
    static std::unique_ptr<cifx::Device> device;
    // Cifx communication channel
    static std::unique_ptr<cifx::Channel> channel;
    // Cifx EtherCAT master
    static std::unique_ptr<cifx::ethercat::Master> master;
    
};

std::string                             CifxEthercatTest::bootloader_path { };
std::string                             CifxEthercatTest::firmware_path   { };
std::string                             CifxEthercatTest::config_path     { };
std::unique_ptr<cifx::Driver>           CifxEthercatTest::driver          { };
std::unique_ptr<cifx::Device>           CifxEthercatTest::device          { };
std::unique_ptr<cifx::Channel>          CifxEthercatTest::channel         { };
std::unique_ptr<cifx::ethercat::Master> CifxEthercatTest::master          { };

/* ==================================================== Toolkit initialization ==================================================== */

TEST_F(CifxEthercatTest, ResourcesFinding) {

    // Get absolute path to the Netx100 bootloader
    ASSERT_NO_THROW(bootloader_path = cifx::get_bootloader_path("NETX100-BSL.bin"));
    // Get absolute path to the EtherCAT netX firmware
    ASSERT_NO_THROW(firmware_path = cifx::ethercat::get_firmware_path("cifxecm.nxf"));
    // Get absolute path to the Velmwheel ENI file
    ASSERT_NO_THROW(config_path = get_test_eni_path().string());

    // Print found resources
    std::cout << log_start << "Following resources has been found" << std::endl;
    std::cout << log_start << "   botloader: " << bootloader_path  << std::endl;
    std::cout << log_start << "   firmware: "  << firmware_path    << std::endl;
    std::cout << log_start << "   eni: "       << config_path      << std::endl;
}

TEST_F(CifxEthercatTest, DriversInitialization) {

    /* -------------------------------- Toolkit setup -------------------------------- */

    // Prepare CIFX driver's configuration (no CoS polling)
    cifx::Driver::Config driver_config {
        .trace_level = cifx::LogLevel::Info
    };
    
    // Initialize Toolkit
    ASSERT_NO_THROW(driver = std::make_unique<cifx::Driver>(driver_config));

    // Name of the CIFX device as identifier in the Toolkit
    constexpr auto CIFX_DEVICE_NAME = "cifx";

    // Prepare CIFX device configuration
    cifx::Device::Config device_config {
        .uio_num = 0,
        .irq_thread_params = {
            .affinity       = 0xFUL,
            .sched_policy   = cifx::ThreadSchedPolicy::Other,
            .sched_priority = 0,
        },
        .bootloader_file = bootloader_path,
        .firmware_file   = firmware_path,
        .config_file     = config_path
    };
    
    // Initialize device
    ASSERT_NO_THROW(device = std::make_unique<cifx::Device>(*driver, CIFX_DEVICE_NAME, device_config));

    // Index of the CIFX communication channel used for EtherCAT communication
    constexpr uint32_t CIFX_CHANNEL_IDX = 0;

    // Initialize channel
    ASSERT_NO_THROW(channel = std::make_unique<cifx::Channel>(*device, CIFX_CHANNEL_IDX));

    /* ---------------------------- EtherCAT driver setup ---------------------------- */

    // Create EtherCAT master driver
    ASSERT_NO_THROW(master = std::make_unique<cifx::ethercat::Master>(*channel));

}

/* ================================================= Test CIFX-specific Master API ================================================ */

TEST_F(CifxEthercatTest, GetMasterStateInfo) {

    cifx::ethercat::Master::StateInfo state_info;

    // Read Master state
    ASSERT_NO_THROW(state_info = master->get_state_info());

    // Print state info
    std::cout << log_start << "Current master state info: "                                                                  << std::endl;
    std::cout << log_start << "  - current state: "        << cifx::ethercat::Master::state_to_str(state_info.target_state)  << std::endl;
    std::cout << log_start << "  - target state: "         << cifx::ethercat::Master::state_to_str(state_info.current_state) << std::endl;
    std::cout << log_start << "  - stop reason: "          << state_info.stop_reason                                         << std::endl;
    std::cout << log_start << "  - slave not in op: "      << state_info.flags.at_least_one_mandatory_slave_not_in_op        << std::endl;
    std::cout << log_start << "  - DC xrmw stopped: "      << state_info.flags.dc_xrmw_stopped                               << std::endl;
    std::cout << log_start << "  - mandatory slave lost: " << state_info.flags.at_least_one_mandatory_slave_lost             << std::endl;
    
}

TEST_F(CifxEthercatTest, SetMasterBusSyncMode) {

    // Set synchronisation mode to IO1
    ASSERT_NO_THROW(master->set_sync_mode(cifx::ethercat::Master::SyncMode::IO1));
    
}

TEST_F(CifxEthercatTest, GetBusTimingInfo) {

    cifx::ethercat::Master::TimingInfo timing_info;

    // Read Master state
    ASSERT_NO_THROW(timing_info = master->get_timing_info());

    // Print state info
    std::cout << log_start << "Current timing parameters: "                                                 << std::endl;
    std::cout << log_start << "  - bus cycle: "               << timing_info.bus_cycle.count()              << std::endl;
    std::cout << log_start << "  - frame transmission time: " << timing_info.frame_transmition_time.count() << std::endl;
    std::cout << log_start << "  - expected bus delay: "      << timing_info.expected_bus_delay.count()     << std::endl;
    std::cout << log_start << "  - expected RX end time: "    << timing_info.expected_rx_end_time.count()   << std::endl;
    std::cout << log_start << "  - expected TX end time: "    << timing_info.expected_tx_end_time.count()   << std::endl;
    
}

/* ==================================================== Test common Master API ==================================================== */

TEST_F(CifxEthercatTest, GetSlavesList) {

    std::vector<std::string_view> slaves;

    // Set synchronisation mode to IO1
    ASSERT_NO_THROW(slaves = master->list_slaves());
    
    auto slave_in_list = [&slaves](std::string_view name) {
        return std::find_if(slaves.begin(), slaves.end(),
            [&name](auto &s){ return s == name; }
        ) != slaves.end();
    };

    // Check if all slaves are listed by the configuration
    ASSERT_EQ  (slaves.size(), 5UL              );
    ASSERT_TRUE(slave_in_list("Imu")            );
    ASSERT_TRUE(slave_in_list("WheelRearLeft")  );
    ASSERT_TRUE(slave_in_list("WheelRearRight") );
    ASSERT_TRUE(slave_in_list("WheelFrontLeft") );
    ASSERT_TRUE(slave_in_list("WheelFrontRight"));
    
}

TEST_F(CifxEthercatTest, GetSlaves) {

    // Check if all slaves can be referenced
    ASSERT_NO_THROW(master->get_slave("Imu")            );
    ASSERT_NO_THROW(master->get_slave("WheelRearLeft")  );
    ASSERT_NO_THROW(master->get_slave("WheelRearRight") );
    ASSERT_NO_THROW(master->get_slave("WheelFrontLeft") );
    ASSERT_NO_THROW(master->get_slave("WheelFrontRight"));
    
}

TEST_F(CifxEthercatTest, RegisterMasterEventHandlers) {

    // Check if all slaves can be referenced
    ASSERT_NO_THROW(master->register_event_handler(cifx::ethercat::Master::Event::ReadBusStart,                 [](){ std::cout << log_start << "Master event: " << "ReadBusStart"                 << std::endl; }));
    ASSERT_NO_THROW(master->register_event_handler(cifx::ethercat::Master::Event::ReadBusComplete,              [](){ std::cout << log_start << "Master event: " << "ReadBusComplete"              << std::endl; }));
    ASSERT_NO_THROW(master->register_event_handler(cifx::ethercat::Master::Event::ReadBusSlavesUpdateComplete,  [](){ std::cout << log_start << "Master event: " << "ReadBusSlavesUpdateComplete"  << std::endl; }));
    ASSERT_NO_THROW(master->register_event_handler(cifx::ethercat::Master::Event::WriteBusStart,                [](){ std::cout << log_start << "Master event: " << "WriteBusStart"                << std::endl; }));
    ASSERT_NO_THROW(master->register_event_handler(cifx::ethercat::Master::Event::WriteBusSlavesUpdateComplete, [](){ std::cout << log_start << "Master event: " << "WriteBusSlavesUpdateComplete" << std::endl; }));
    ASSERT_NO_THROW(master->register_event_handler(cifx::ethercat::Master::Event::WriteBusComplete,             [](){ std::cout << log_start << "Master event: " << "WriteBusComplete"             << std::endl; }));
    
}

TEST_F(CifxEthercatTest, UnregisterMasterEventHandlers) {

    // Check if all slaves can be referenced
    ASSERT_NO_THROW(master->unregister_event_handler(cifx::ethercat::Master::Event::ReadBusStart                ));
    ASSERT_NO_THROW(master->unregister_event_handler(cifx::ethercat::Master::Event::ReadBusComplete             ));
    ASSERT_NO_THROW(master->unregister_event_handler(cifx::ethercat::Master::Event::ReadBusSlavesUpdateComplete ));
    ASSERT_NO_THROW(master->unregister_event_handler(cifx::ethercat::Master::Event::WriteBusStart               ));
    ASSERT_NO_THROW(master->unregister_event_handler(cifx::ethercat::Master::Event::WriteBusSlavesUpdateComplete));
    ASSERT_NO_THROW(master->unregister_event_handler(cifx::ethercat::Master::Event::WriteBusComplete            ));
    
}

TEST_F(CifxEthercatTest, GetMasterState) {

    cifx::ethercat::Master::State state;

    // Read Master state
    ASSERT_NO_THROW(state = master->get_state());

    // Print state info
    std::cout << log_start << "Current master state: " << cifx::ethercat::Master::state_to_str(state) << std::endl;
    
}

TEST_F(CifxEthercatTest, SetMasterState) {

    // Set synchronisation mode to IO1
    ASSERT_NO_THROW(master->set_state(cifx::ethercat::Master::State::Safeop));

    std::this_thread::sleep_for(std::chrono::seconds{ 1 });

    // Set synchronisation mode to IO1
    ASSERT_NO_THROW(master->set_state(cifx::ethercat::Master::State::Preop));
    
}

TEST_F(CifxEthercatTest, ReadBus) {

    // Read bus
    ASSERT_NO_THROW(master->read_bus(std::chrono::milliseconds{ 1'000 }));
    
}

TEST_F(CifxEthercatTest, WriteBus) {
    
    // Write bus
    ASSERT_NO_THROW(master->write_bus(std::chrono::milliseconds{ 1'000 }));
    
}

/* ==================================================== Toolkit initialization ==================================================== */

TEST_F(CifxEthercatTest, DriversDeinitialization) {

    // Unregister slave from the master
    ASSERT_NO_THROW(master.reset());

    // Cleanup communication channel
    ASSERT_NO_THROW(channel.reset());
    // Cleanup device driver
    ASSERT_NO_THROW(device.reset());
    // Close the toolkit driver
    ASSERT_NO_THROW(driver.reset());

}

/* ================================================================================================================================ */