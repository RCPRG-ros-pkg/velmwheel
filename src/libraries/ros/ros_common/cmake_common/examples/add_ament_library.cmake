# ====================================================================================================================================
# @file       add_ament_library.cmake
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Thursday, 12th May 2022 11:27:46 pm
# @modified   Tuesday, 24th May 2022 8:56:39 pm
# @project    engineering-thesis
# @brief      Usage example of the add_ament_library() macro
#    
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# Linkable dependencies
list(APPEND LIB_DEPENDENCIES
    Boost    
    rclcpp
)

# Add target (SHARED library)
add_ament_library(

    # Library name
    LIBRARY_NAME ${PROJECT_NAME}_shared

    # Library sources
    SOURCES
        src/src1.cpp
        src/src2.cpp

    # Additional includes (except ${CMAKE_CURRENT_SOURCE_DIR}/include)
    ADDITIONAL_INCLUDES
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/custom_include>
        $<INSTALL_INTERFACE:include/custom_include>

    # Dependencies
    AMENT_DEPENDENCIES ${LIB_DEPENDENCIES}
    
)

# Add target (INTERFACE library)
add_ament_library(

    # Library name
    LIBRARY_NAME ${PROJECT_NAME}_interface
    # Library type
    LIBRARY_TYPE INTERFACE

    # Dependencies
    AMENT_DEPENDENCIES ${LIB_DEPENDENCIES}
    
)
