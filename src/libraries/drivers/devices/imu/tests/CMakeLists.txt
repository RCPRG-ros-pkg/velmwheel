# ====================================================================================================================================
# @file       CMakeLists.txt
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Wednesday, 13th April 2021 12:43:42 am
# @modified   Tuesday, 21st June 2022 6:20:41 pm
# @project    engineering-thesis
# @brief      Build rules for tests of the abstract EtherCAT driver of the IMU servodriver
#    
#
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ========================================================== Dependencies ========================================================== #

# Find GTest package
find_package(ament_cmake_gtest_ext REQUIRED)
# Find additional libraries
find_package(package_common_cpp REQUIRED)

# ========================================================== Configuration ========================================================= #

# Common suffix for running all tests
list(APPEND COMMON_SUFFIX
    --gtest_break_on_failure
)

# ======================================================= DefaultParse tests ======================================================= #

set(TEST_NAME imu_test)

# Add test
ament_add_gtest(${TEST_NAME}

    # Test sources
    src/imu_test.cpp

    # Test-runner suffix (stop test-case after first failure)
    COMMAND_SUFFIX ${COMMON_SUFFIX}
    
)

# Add include directories
target_include_directories(${TEST_NAME}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Link dependencies to the test executable [ament packages]
ament_target_dependencies(${TEST_NAME}
    package_common_cpp
)

# Link dependencies to the test executable [targets]
target_link_libraries(${TEST_NAME}
    ${PROJECT_NAME}
)

# Additional compilation flags for the test
target_compile_options(${TEST_NAME}
    PRIVATE
        -Wno-maybe-uninitialized
)

# ============================================================ Resources =========================================================== #

# Install shared resources
install(
    DIRECTORY
        data
    DESTINATION
        share/${PROJECT_NAME}/test
)

# Register ament resources
ament_index_register_resource(test_data CONTENT share/${PROJECT_NAME}/test/data)

# ================================================================================================================================== #
