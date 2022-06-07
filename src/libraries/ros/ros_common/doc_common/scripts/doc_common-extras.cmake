# ====================================================================================================================================
# @file       doc_common-extras.cmake
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Monday, 23rd May 2022 9:49:48 pm
# @modified   Wednesday, 25th May 2022 4:44:01 pm
# @project    engineering-thesis
# @brief      Package's resource file adding resources imported by downstream packages when the `doc_common` package is found
#    
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# Check if COLCON_PREFIX_PATH variable is set
if(NOT ENV{COLCON_PREFIX_PATH})
    
    # ----------------------------------------------------------------------------------------
    # If path is not set, it means that no package has been built yet in a way that made
    # user able to source `install/setup.bash` file. But as the `doc_common` package itself
    # must have been built (as the CMake is processing current code), we can obtain the
    # COLCON_PREFIX_PATH variable in a 'hacky' way based on the installation dir of the
    # package.
    # ----------------------------------------------------------------------------------------
    set(ENV{COLCON_PREFIX_PATH} "${doc_common_DIR}/../../../..")
    
endif()

# Path to the colcon build dir
set(COLCON_LOG_DIR $ENV{COLCON_PREFIX_PATH}/../log)
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
