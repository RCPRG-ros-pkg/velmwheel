# ====================================================================================================================================
# @file       CMakeLists.txt
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Wednesday, 13th April 2021 12:43:42 am
# @modified   Friday, 27th May 2022 1:33:24 am
# @project    engineering-thesis
# @brief
#    
#    Build rules for Hilscher's CIFX/netX Toolkit-based CIFX EtherCAT tests
#
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ========================================================== Dependencies ========================================================== #

# Find GTest package
find_package(ament_cmake_gtest_ext REQUIRED)

# ========================================================== Configuration ========================================================= #

# Common suffix for running all tests
list(APPEND COMMON_SUFFIX
    --gtest_break_on_failure
)

# ============================================================ Main test =========================================================== #

set(TEST_NAME ethercat_test)

# Add test
ament_add_gtest(${TEST_NAME}

    # Test sources
    src/test.cpp

    # Test-runner suffix (stop test-case after first failure)
    COMMAND_SUFFIX ${COMMON_SUFFIX}
    
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
