# ====================================================================================================================================
# @file       CMakeLists.txt
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Wednesday, 16th March 2022 4:33:51 pm
# @modified   Wednesday, 25th May 2022 4:44:01 pm
# @project    engineering-thesis
# @brief      CMake file for `node_common_cpp` package
#    
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Project ============================================================ #

# CMake version
cmake_minimum_required(VERSION 3.8)
# Project header
project(node_common_cpp)

# ========================================================== Configuration ========================================================= #

# Add extra warnings
add_compile_options(-Wall -Wextra -Wpedantic)

# Require C++ 17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Options
option(BUILD_DOC "If TRUE documentation will be built" ON)

# ========================================================== Dependencies ========================================================== #

# Linkable dependencies
list(APPEND LIB_DEPENDENCIES
    rclcpp
)

# Common dependencies
find_package(cmake_common REQUIRED)
# Dependencies
find_packages(

    # Ament dependency (always present)
    ament_cmake
    # Find dependencies
    ${LIB_DEPENDENCIES}

)

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

# =============================================================== Docs ============================================================= #

# Build doc
if(BUILD_DOC)
    add_subdirectory(doc)
endif()

# ============================================================== Build ============================================================= #

# Build package using ament
ament_package()
