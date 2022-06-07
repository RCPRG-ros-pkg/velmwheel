# ====================================================================================================================================
# @file       CMakeLists.txt
# @author     Dirk Thomas
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Wednesday, 27th April 2022 6:50:23 pm
# @modified   Wednesday, 25th May 2022 4:44:01 pm
# @project    engineering-thesis
# @brief      Main CMake file of the package
#    
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Project ============================================================ #

# CMake version
cmake_minimum_required(VERSION 3.12)
# Project header
project(ament_cmake_gtest_ext NONE)

# ========================================================== Dependencies ========================================================== #

# Ament dependenies
find_package(ament_cmake_core REQUIRED)

# ========================================================= Shared installs ======================================================== #

# Install shared resources
install(
    DIRECTORY
        cmake
    DESTINATION
        share/${PROJECT_NAME}
)

# ============================================================== Build ============================================================= #

# Build package
ament_package(CONFIG_EXTRAS "scripts/ament_cmake_gtest_ext-extras.cmake")
