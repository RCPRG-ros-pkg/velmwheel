# ====================================================================================================================================
# @file       CMakeLists.txt
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Tuesday, 1st March 2022 4:30:53 pm
# @modified   Wednesday, 25th May 2022 4:44:01 pm
# @project    engineering-thesis
# @brief      CMake file for `velmwheel_twincat` package
#    
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Project ============================================================ #

# CMake version
cmake_minimum_required(VERSION 3.8)
# Project header
project(velmwheel_twincat)

# ========================================================== Dependencies ========================================================== #

# Ament dependency (always present)
find_package(ament_cmake REQUIRED)

# ============================================================ Installs ============================================================ #

# Install shared resources
install(
    DIRECTORY
        eni
        project
    DESTINATION
        share/${PROJECT_NAME}
)

# Register ament resources
ament_index_register_resource(eni     CONTENT share/${PROJECT_NAME}/eni)
ament_index_register_resource(project CONTENT share/${PROJECT_NAME}/project)

# ============================================================== Build ============================================================= #

# Build package using ament
ament_package()
