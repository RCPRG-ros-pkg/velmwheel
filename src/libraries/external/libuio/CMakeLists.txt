# ====================================================================================================================================
# @file       CMakeLists.txt
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Wednesday, 13th April 2021 12:43:42 am
# @modified   Friday, 13th May 2022 5:41:17 pm
# @project    engineering-thesis
# @brief
#    
#    Build rules for libuio library
#
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Project ============================================================ #

# CMake version
cmake_minimum_required(VERSION 3.5)
# Project header
project(libuio VERSION 0.2.8)

# ========================================================== Configuration ========================================================= #

# Compilation options
add_compile_options(
    -fstack-protector-strong
    -Wno-unused-parameter
    -Wno-format-truncation
    -Werror=format-security
    -Wformat
    -g
    -O2
)

# Configuration header
configure_file(config.h.in config.h)

# Compilation proprties
set(CXX_STANDARD 17)
set(CXX_EXTENSIONS ON)
set(CXX_STANDARD_REQUIRED ON)
set(C_STANDARD 11)
set(C_EXTENSIONS ON)
set(C_STANDARD_REQUIRED ON)

# =========================================================== Dependancies ========================================================= #

# Common dependencies
find_package(cmake_common REQUIRED)
# Find dependencies
find_package(ament_cmake REQUIRED)

# ========================================================= Library target ========================================================= #

# Add target
add_ament_library(

    # Library name
    LIBRARY_NAME uio

    # Library sources
    SOURCES
        src/base.c
        src/helper.c
        src/irq.c
        src/mem.c
        src/attr.c

    # Additional includes
    ADDITIONAL_INCLUDES
        PRIVATE
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
            $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
            
    # Dependencies
    AMENT_DEPENDENCIES ${LIB_DEPENDENCIES}
    
)

# =================================================== Executable target (lsuio) ==================================================== #

# lsuio's source files
add_executable(lsuio
    src/lsuio.c
)

# Dependancy on libuio
target_link_libraries(lsuio
    PRIVATE
        uio
)

# Includes' path
target_include_directories(lsuio
    PRIVATE
        "${PROJECT_BINARY_DIR}"
)

# ================================================== Executable target (readuio) =================================================== #

# lsuio's source files
add_executable(readuio
    src/readuio.c
)

# Dependancy on libuio
target_link_libraries(readuio
    PRIVATE
        uio
)

# Includes' path
target_include_directories(readuio
    PRIVATE
        "${PROJECT_BINARY_DIR}"
)

# ============================================================= Installs =========================================================== #

# Creat export set
install(
    TARGETS lsuio readuio
    EXPORT uio-export
    LIBRARY  DESTINATION lib
    ARCHIVE  DESTINATION lib
    RUNTIME  DESTINATION bin
    INCLUDES DESTINATION include
)

# Export targets
ament_export_targets(uio-export)

# ============================================================== Building =========================================================== #

# Build package
ament_package()
