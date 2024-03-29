# ====================================================================================================================================
# @file       ament_cmake_gtest_ext-extras.cmake
# @author     Dirk Thomas
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Wednesday, 27th April 2022 6:50:23 pm
# @modified   Monday, 11th July 2022 3:57:41 pm
# @project    engineering-thesis
# @brief      Modified version of the `_ament_cmake_gtest_find_gtest` macro
#    
# Copyright 2014-2015 Open Source Robotics Foundation, Inc.
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

# ============================================================= Macros ============================================================= #

# ---------------------------------------------------------------------------------------
# @brief Finds gtest and create library targets once
# ---------------------------------------------------------------------------------------
macro(_ament_cmake_gtest_find_gtest)

    # If GTest not found yet
    if(NOT DEFINED _AMENT_CMAKE_GTEST_FIND_GTEST)

        # Mark GTest as found
        set(_AMENT_CMAKE_GTEST_FIND_GTEST TRUE)

        # Find ament helper package
        find_package(ament_cmake_test QUIET REQUIRED)

        # If gtest sources were not found in a previous run
        if(NOT GTEST_FROM_SOURCE_FOUND)
        
            # Search path for gtest includes and sources
            set(_search_path_include "")
            set(_search_path_src "")

            # `option()` consider environment variable to find gtest
            if(NOT $ENV{GTEST_DIR} STREQUAL "")
                list(APPEND _search_path_include "$ENV{GTEST_DIR}/include/gtest")
                list(APPEND _search_path_src "$ENV{GTEST_DIR}/src")
            endif()

            # Check to system installed path (i.e. on Ubuntu)
            set(_search_path_include "/usr/include/gtest")
            set(_search_path_src "/usr/src/gtest/src")

            # Check gtest_vendor path, prefer this version over a system installed
            find_package(gtest_vendor QUIET)
            if(gtest_vendor_FOUND AND gtest_vendor_BASE_DIR)
                list(INSERT _search_path_include 0 "${gtest_vendor_BASE_DIR}/include/gtest")
                list(INSERT _search_path_src 0 "${gtest_vendor_BASE_DIR}/src")
            endif()

            # Find main header of GTest
            find_file(_gtest_header_file "gtest.h"
                PATHS ${_search_path_include}
                NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH
            )
            # Find main sources of GTest
            find_file(_gtest_src_file
                "gtest.cc"
                "gtest-all.cc"
                PATHS ${_search_path_src}
                NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH
            )

            # If both found
            if(_gtest_header_file AND _gtest_src_file)

                # Set from-source variables
                set(GTEST_FROM_SOURCE_FOUND TRUE CACHE INTERNAL "")

                # Compute intermediate strings
                get_filename_component(_gtest_base_dir "${_gtest_src_file}" PATH)
                get_filename_component(_gtest_base_dir "${_gtest_base_dir}" PATH)
                get_filename_component(_gtest_include_dir "${_gtest_header_file}" PATH)
                get_filename_component(_gtest_include_dir ${_gtest_include_dir} PATH)

                # Set relevent variables
                set(GTEST_FROM_SOURCE_BASE_DIR "${_gtest_base_dir}" CACHE INTERNAL "")
                set(GTEST_FROM_SOURCE_INCLUDE_DIRS "${_gtest_include_dir}" CACHE INTERNAL "")
                set(GTEST_FROM_SOURCE_LIBRARY_DIRS "${CMAKE_BINARY_DIR}/gtest" CACHE INTERNAL "")
                set(GTEST_FROM_SOURCE_LIBRARIES "gtest" CACHE INTERNAL "")
                set(GTEST_FROM_SOURCE_MAIN_LIBRARIES "gtest_main" CACHE INTERNAL "")

            endif()
            
        endif()

        # If GTest found
        if(GTEST_FROM_SOURCE_FOUND)

            message(STATUS
                "Found gtest sources under '${GTEST_FROM_SOURCE_BASE_DIR}': "
                "C++ tests using 'Google Test' will be built"
            )

            # If the gtest sources are from a subfolder of gmock (not the case for gmock_vendor)
            # the subdirectory was already added by gmock
            if(NOT TARGET gtest)

                # Add CMakeLists.txt from gtest dir
                add_subdirectory("${GTEST_FROM_SOURCE_BASE_DIR}" "${CMAKE_BINARY_DIR}/gtest")

                # Mark gtest targets with EXCLUDE_FROM_ALL to only build when tests are built which depend on them
                set_target_properties(gtest gtest_main PROPERTIES EXCLUDE_FROM_ALL 1)
                # Add platform-specific options
                if(NOT WIN32)
                    target_compile_options(gtest PRIVATE -Wno-null-dereference)
                endif()
                # Add include directories
                target_include_directories(gtest BEFORE PUBLIC "${GTEST_FROM_SOURCE_INCLUDE_DIRS}")
                target_include_directories(gtest_main BEFORE PUBLIC "${GTEST_FROM_SOURCE_INCLUDE_DIRS}")

            endif()

            # Set the same variables as find_package() but do NOT set GTEST_FOUND in the cache when using gtest from source
            # since the subdirectory must always be added to add the gtest targets
            set(GTEST_FOUND ${GTEST_FROM_SOURCE_FOUND})
            set(GTEST_INCLUDE_DIRS ${GTEST_FROM_SOURCE_INCLUDE_DIRS})
            set(GTEST_LIBRARY_DIRS ${GTEST_FROM_SOURCE_LIBRARY_DIRS})
            set(GTEST_LIBRARIES ${GTEST_FROM_SOURCE_LIBRARIES})
            set(GTEST_MAIN_LIBRARIES ${GTEST_FROM_SOURCE_MAIN_LIBRARIES})
            set(GTEST_BOTH_LIBRARIES ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES})
            
        endif()

        # If failed to find GTest
        if(NOT GTEST_FOUND)
            message(WARNING
                "'gtest' not found, C++ tests using 'Google Test' can not be built. "
                "Please install the 'Google Test' headers globally in your system "
                "to enable these tests (e.g. on Ubuntu/Debian install the package "
                "'libgtest-dev') or get the ament package 'gtest_vendor'")
        endif()
        
    endif()
    
endmacro()

# ============================================================ Includes ============================================================ #

include("${ament_cmake_gtest_ext_DIR}/ament_add_gtest.cmake")
include("${ament_cmake_gtest_ext_DIR}/ament_add_gtest_executable.cmake")
include("${ament_cmake_gtest_ext_DIR}/ament_add_gtest_test.cmake")
include("${ament_cmake_gtest_ext_DIR}/ament_find_gtest.cmake")

# ================================================================================================================================== #
