# ====================================================================================================================================
# @file       CMakeLists.txt
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Wednesday, 16th March 2022 4:33:51 pm
# @modified   Wednesday, 25th May 2022 3:32:59 pm
# @project    engineering-thesis
# @brief      CMake file for `cmake_common` package
#    
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ========================================================== Dependencies ========================================================== #

# Dependencies
find_package(doc_common REQUIRED)

# =============================================================== Docs ============================================================= #

# Add documentation target (Sphinx)
add_sphinx_doc(sphinx_doc

    # Working directory (Sphinx)
    WORKING_DIR ${PROJECT_SOURCE_DIR}
    # Source directory (Sphinx)
    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}
    # Output directory (Sphinx)
    OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/doc/sphinx
    # Install dircetory (Sphinx)
    INSTALL_DIR "share/${PROJECT_NAME}/doc/sphinx"

    # Define additional envs to be used by conf.py
    ENVS MODULE_HOME=${PROJECT_SOURCE_DIR}
    
)

# ================================================================================================================================== #
