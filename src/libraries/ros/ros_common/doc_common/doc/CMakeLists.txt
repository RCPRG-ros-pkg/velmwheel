# ====================================================================================================================================
# @file       CMakeLists.txt
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Wednesday, 16th March 2022 4:33:51 pm
# @modified   Thursday, 9th June 2022 3:43:58 am
# @project    engineering-thesis
# @brief      CMake file for `cmake_common` package
#    
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ========================================================== Dependencies ========================================================== #

# Add cmake dircetory to path, to use find_package(Sphinx)
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

# Find dependencies
find_package(Doxygen REQUIRED)
find_package(Sphinx REQUIRED)

# ============================================================ Includes ============================================================ #

# Include Sphinx helper for documentation generation
include(${PROJECT_SOURCE_DIR}/cmake/add_sphinx_doc.cmake)

# =============================================================== Docs ============================================================= #

# List autodoc source files
file(GLOB_RECURSE AUTODOC_SOURCE_FILES LIST_DIRECTORIES false ${PROJECT_SOURCE_DIR}/doc_common/*.py)
file(GLOB_RECURSE RST_SOURCE_FILES     LIST_DIRECTORIES false ${CMAKE_CURRENT_SOURCE_DIR}/*.rst    )
# List source files
set(DOC_SOURCE_FILES ${RST_SOURCE_FILES} ${AUTODOC_SOURCE_FILES})

# Add documentation target (Sphinx)
add_sphinx_doc(sphinx_doc

    # Working directory (Sphinx)
    WORKING_DIR ${PROJECT_SOURCE_DIR}
    # Source directory (Sphinx)
    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}
    # Source files (Sphinx)
    SOURCE_FILES ${DOC_SOURCE_FILES}
    # Output directory (Sphinx)
    OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/doc/sphinx

    # --------------------------------------------------------------------
    # @brief Log file (Sphinx)
    # @note `doc_common` package has no local dependencies. As so, this
    #    is possible that no other of local packages has been built and
    #    neither COLCON_PREFIX_PATH env variable has been set by sourcing
    #    installed package nor it can be deduced from the installation
    #    path of the `doc_common` itself. As so, package needs to use
    #    non-colcon-standard destination for log file (which is 
    #    <COLCON_RUN_DIR>/log) and instead use in-build-tree destination
    # --------------------------------------------------------------------
    OUTPUT_LOG_FILE ${CMAKE_CURRENT_BINARY_DIR}/log/sphinx

    # Install dircetory (Sphinx)
    INSTALL_DIR "share/${PROJECT_NAME}/doc/sphinx"

    # Define additional envs to be used by conf.py
    ENVS MODULE_HOME=${PROJECT_SOURCE_DIR}
    
)

# ================================================================================================================================== #
