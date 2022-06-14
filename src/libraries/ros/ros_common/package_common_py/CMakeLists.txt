# ====================================================================================================================================
# @file       CMakeLists.txt
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Tuesday, 14th June 2022 2:19:10 pm
# @modified   Tuesday, 14th June 2022 2:19:12 pm
# @project    engineering-thesis
# @brief      CMake file for `package_common_py` package
#    
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Project ============================================================ #

# CMake version
cmake_minimum_required(VERSION 3.8)
# Project header
project(package_common_py)

# ========================================================== Configruation ========================================================= #

# Options
option(BUILD_DOC "If TRUE documentation will be built" ON)

# ========================================================== Dependencies ========================================================== #

# Ament dependency (always present)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# ===================================================== Source (Python nodes) ====================================================== #

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# =============================================================== Docs ============================================================= #

# Build doc
if(BUILD_DOC)
    add_subdirectory(doc)
endif()

# ============================================================== Build ============================================================= #

# Build package using ament
ament_package()
