/* ============================================================================================================================ *//**
 * @file       test.cpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 18th May 2021 1:54:23 am
 * @modified   Friday, 27th May 2022 3:42:15 pm
 * @project    engineering-thesis
 * @brief      Unit tests for object-oriented interface fo the driver. Test is desire to run on machine with CIFX card up and running.
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

/* =========================================================== Includes =========================================================== */

// System includes
#include <memory>
// Test includes
#include "gtest/gtest.h"
// Private includes
#include "cifx.hpp"

/* ============================================================= Tests ============================================================ */

TEST(CifxDriver, CppTest) {

    // Prepare CIFX driver's configuration
    cifx::Driver::Config driver_config = {
        .cos_polling_interval_ms = std::optional<std::chrono::milliseconds>{ },
        .cos_polling_thread_params = {
            .affinity = 0xf,
            .sched_policy = cifx::ThreadSchedPolicy::Other,
            .sched_priority = 0
        },
        .trace_level = cifx::LogLevel::Info
    };

    // Initialize Toolkit
    std::unique_ptr<cifx::Driver> driver;
    ASSERT_NO_THROW(driver = std::make_unique<cifx::Driver>(driver_config));

    // Prepare CIFX device configuration
    cifx::Device::Config device_config = {
        .uio_num = 0,
        .irq_thread_params = {
            .affinity = 0xf,
            .sched_policy = cifx::ThreadSchedPolicy::Other,
            .sched_priority = 0
        },
        .bootloader_file = "install/cifx_toolkit/bootloaders/NETX100-BSL.bin",
        .firmware_file = "install/cifx_ethercat/firmware/cifxecm.nxf",
        .config_file = "none",
    };

    // Initialize device
    std::unique_ptr<cifx::Device> device;
    ASSERT_NO_THROW(device = std::make_unique<cifx::Device>(*driver, "cifx", device_config));

    // Open the driver's handle
    std::unique_ptr<cifx::Channel> channel;
    ASSERT_NO_THROW(channel = std::make_unique<cifx::Channel>(*device, 0));

    // Close the channel
    ASSERT_NO_THROW(channel.reset());
    // Close the driver connection
    ASSERT_NO_THROW(device.reset());
    // Close the Toolkit explicitly
    ASSERT_NO_THROW(driver.reset());
}

/* ================================================================================================================================ */