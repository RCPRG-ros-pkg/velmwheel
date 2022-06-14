# ====================================================================================================================================
# @file       doc_common-extras.cmake
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Monday, 23rd May 2022 9:49:48 pm
# @modified   Thursday, 9th June 2022 4:45:19 am
# @project    engineering-thesis
# @brief      Package's resource file adding resources imported by downstream packages when the `doc_common` package is found
#    
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================
    
# Path to the default log dir dir
if(DEFINED ENV{COLCON_PREFIX_PATH})
    set(DOC_COMMON_LOG_DIR $ENV{COLCON_PREFIX_PATH}/../log)
else()
    set(DOC_COMMON_LOG_DIR ${CMAKE_BINARY_DIR}/log)
endif()
# Add cmake dircetory to path, to use find_package(Sphinx)
set(CMAKE_MODULE_PATH "${doc_common_DIR}" ${CMAKE_MODULE_PATH})
# Set auxiliary variables
set(DOC_COMMON_TEMPLATES_DIR "${doc_common_DIR}/../templates")

# Find dependencies
find_package(Doxygen REQUIRED)
find_package(Sphinx REQUIRED)

# Include library
include("${doc_common_DIR}/add_doxygen_doc.cmake")
include("${doc_common_DIR}/add_sphinx_doc.cmake")
