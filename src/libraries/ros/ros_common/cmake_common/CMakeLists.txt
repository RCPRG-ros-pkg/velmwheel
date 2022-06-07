# ====================================================================================================================================
# @file       CMakeLists.txt
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
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
project(cmake_common NONE)

# ========================================================== Configruation ========================================================= #

# Options
option(BUILD_DOC "If TRUE documentation will be built" ON)

# ========================================================== Dependencies ========================================================== #

# Find dependenies
find_package(ament_cmake REQUIRED)
# Export dependenies
ament_export_dependencies(ament_cmake)

# ========================================================= Shared installs ======================================================== #

# Install shared resources
install(
    DIRECTORY
        cmake
    DESTINATION
        share/${PROJECT_NAME}
)

# =============================================================== Docs ============================================================= #

# Build doc
if(BUILD_DOC)
    add_subdirectory(doc)
endif()

# ============================================================== Build ============================================================= #

# Build package
ament_package(CONFIG_EXTRAS "scripts/cmake_common-extras.cmake")
