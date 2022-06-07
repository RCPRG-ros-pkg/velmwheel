# ====================================================================================================================================
# @file       CMakeLists.txt
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Wednesday, 16th March 2022 4:33:51 pm
# @modified   Wednesday, 25th May 2022 4:44:01 pm
# @project    engineering-thesis
# @brief      CMake file for `itsy_bitsy` package
#    
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Project ============================================================ #

# CMake version
cmake_minimum_required(VERSION 3.8)
# Project header
project(itsy_bitsy)

# ========================================================== Configuration ========================================================= #

# Add extra warnings
add_compile_options(-Wall -Wextra -Wpedantic)

# Require C++ 17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# ========================================================== Dependencies ========================================================== #

# Common dependencies
find_package(cmake_common REQUIRED)
# Ament dependency (always present)
find_package(ament_cmake REQUIRED)

# ============================================================= Source ============================================================= #

# Add target
add_ament_library(

    # Library name
    LIBRARY_NAME ${PROJECT_NAME}
    # Library type
    LIBRARY_TYPE INTERFACE

    # Dependencies
    AMENT_DEPENDENCIES ${LIB_DEPENDENCIES}
    
)

# ============================================================== Tests ============================================================= #

# Test builds
if(BUILD_TESTING)

    # Find linter
    find_package(ament_lint_auto REQUIRED)
    # Find test dependencies using info from `package.xml`
    ament_lint_auto_find_test_dependencies()

endif()

# ============================================================== Build ============================================================= #

# Build package using ament
ament_package()
