# ====================================================================================================================================
# @file       ament_add_gtest.cmake
# @author     Dirk Thomas
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Wednesday, 27th April 2022 6:50:23 pm
# @modified   Monday, 11th July 2022 3:56:45 pm
# @project    engineering-thesis
# @brief      Modified version of the `ament_add_gtest_test` function providing ability to add prefix and suffix strings to the test 
#             run command
#    
# Copyright 2014-2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ====================================================================================================================================

# ---------------------------------------------------------------------------------------
# @brief Add an existing executable using gtest as a test.
#
# @details Register an executable created with ament_add_gtest_executable() as a test.
#    If the specified target does not exist the registration is skipped.
#
# @param target [NAME]
#    the target name which will also be used as the test name
# @param RUNNER [STR]
#    the path to the test runner script (default: see ament_add_test).
# @param TIMEOUT [INT]
#    the test timeout in seconds
# @param WORKING_DIRECTORY [PATH]
#    the working directory for invoking the executable in, default defined by 
#    ``ament_add_test()``
# @param SKIP_TEST
#    if set mark the test as being skipped
# @param ENV [NAME=VALUE...]
#    list of env vars to set; listed as ``VAR=value``
# @param APPEND_ENV [NAME=VALUE...]
#    list of env vars to append if already set, otherwise set;
#    listed as ``VAR=value``
# @param APPEND_LIBRARY_DIRS [NAMES...]
#    list of library dirs to append to the appropriate
#    OS specific env var, a la LD_LIBRARY_PATH
#
# @added
#
# @param COMMAND_PREFIX 
#    multielement keyword argument providing strings to prefix test command with
# @param COMMAND_SUFFIX 
#    multielement keyword argument providing strings to suffix test command with
#   
# @endadded
# ---------------------------------------------------------------------------------------
function(ament_add_gtest_test target)
    
    # Check if target given
    if(NOT TARGET ${target})
        return()
    endif()

    # Parse arguments
    cmake_parse_arguments(ARG
        "SKIP_TEST"
        "RUNNER;TIMEOUT;WORKING_DIRECTORY"
        "APPEND_ENV;APPEND_LIBRARY_DIRS;ENV;COMMAND_PREFIX;COMMAND_SUFFIX"
        ${ARGN}
    )

    # Check if unused arguments given
    if(ARG_UNPARSED_ARGUMENTS)
        message(FATAL_ERROR "ament_add_gtest_test() called with unused arguments: ${ARGN}")
    endif()

    # Prepare path to the executable
    set(executable "$<TARGET_FILE:${target}>")
    # Prepare path to the result file
    set(result_file "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/${target}.gtest.xml")
    # Prepare command to run test
    set(cmd
        "${ARG_COMMAND_PREFIX}"
        "${executable}"
        "--gtest_output=xml:${result_file}"
        "${ARG_COMMAND_SUFFIX}"
    )
    
    # Prepend keyword argument with keyword (if given)
    if(ARG_ENV)
        set(ARG_ENV "ENV" ${ARG_ENV})
    endif()
    if(ARG_APPEND_ENV)
        set(ARG_APPEND_ENV "APPEND_ENV" ${ARG_APPEND_ENV})
    endif()
    if(ARG_APPEND_LIBRARY_DIRS)
        set(ARG_APPEND_LIBRARY_DIRS "APPEND_LIBRARY_DIRS" ${ARG_APPEND_LIBRARY_DIRS})
    endif()
    if(ARG_RUNNER)
        set(ARG_RUNNER "RUNNER" ${ARG_RUNNER})
    endif()
    if(ARG_TIMEOUT)
        set(ARG_TIMEOUT "TIMEOUT" ${ARG_TIMEOUT})
    endif()
    if(ARG_WORKING_DIRECTORY)
        set(ARG_WORKING_DIRECTORY "WORKING_DIRECTORY" "${ARG_WORKING_DIRECTORY}")
    endif()
    if(ARG_SKIP_TEST)
        set(ARG_SKIP_TEST "SKIP_TEST")
    endif()

    # Add test
    ament_add_test(
        "${target}"
        COMMAND ${cmd} 
        OUTPUT_FILE "${CMAKE_BINARY_DIR}/ament_cmake_gtest/${target}.txt"
        RESULT_FILE "${result_file}"
        ${ARG_RUNNER}
        ${ARG_SKIP_TEST}
        ${ARG_ENV}
        ${ARG_APPEND_ENV}
        ${ARG_APPEND_LIBRARY_DIRS}
        ${ARG_TIMEOUT}
        ${ARG_WORKING_DIRECTORY}
    )

    # Set additional properties on the test
    set_tests_properties(
        "${target}"
        PROPERTIES
        REQUIRED_FILES "${executable}"
        LABELS "gtest"
    )
    
endfunction()
