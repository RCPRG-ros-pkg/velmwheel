# ====================================================================================================================================
# @file       CMakeLists.txt
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Monday, 23rd May 2022 9:43:25 pm
# @modified   Wednesday, 25th May 2022 2:04:28 pm
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
project(doc_common NONE)

# ========================================================== Configruation ========================================================= #

# Options
option(BUILD_DOC "If TRUE documentation will be built" ON)

# ========================================================== Dependencies ========================================================== #

# Find dependenies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
# Export dependenies
ament_export_dependencies(ament_cmake)
ament_export_dependencies(ament_cmake_python)

# ===================================================== Source (Python nodes) ====================================================== #

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# ========================================================= Shared installs ======================================================== #

# Install shared resources
install(
    DIRECTORY
        cmake
        templates
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
ament_package(CONFIG_EXTRAS "scripts/doc_common-extras.cmake")
