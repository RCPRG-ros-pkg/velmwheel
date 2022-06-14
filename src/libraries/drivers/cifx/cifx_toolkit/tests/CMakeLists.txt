# ====================================================================================================================================
# @file       CMakeLists.txt
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Wednesday, 13th April 2021 12:43:42 am
# @modified   Friday, 20th May 2022 8:24:03 pm
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

# ======================================================== ENi parsing test ======================================================== #

set(TEST_NAME cpp_test)

# Add test
ament_add_gtest(${TEST_NAME}

    # Test sources
    src/test.cpp

    # Test-runner suffix (stop test-case after first failure)
    COMMAND_SUFFIX ${COMMON_SUFFIX}
    
)

# Includes' pathethercat_test
target_include_directories(${TEST_NAME}
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
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

# ================================================================================================================================== #
