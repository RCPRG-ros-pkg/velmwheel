# ====================================================================================================================================
# @file       CMakeLists.txt
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Wednesday, 16th March 2022 4:33:51 pm
# @modified   Tuesday, 14th June 2022 2:45:14 pm
# @project    engineering-thesis
# @brief      CMake file for `package_common_py` package
#    
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ========================================================== Dependencies ========================================================== #

# Dependencies
find_package(doc_common REQUIRED)

# =============================================================== Docs ============================================================= #

# List autodoc source files
file(GLOB_RECURSE AUTODOC_SOURCE_FILES LIST_DIRECTORIES false ${PROJECT_SOURCE_DIR}/${PROJECT_NAME}/*.py)
file(GLOB_RECURSE RST_SOURCE_FILES     LIST_DIRECTORIES false ${CMAKE_CURRENT_SOURCE_DIR}/*.rst         )
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
    # Install dircetory (Sphinx)
    INSTALL_DIR "share/${PROJECT_NAME}/doc/sphinx"

    # Define additional envs to be used by conf.py
    ENVS MODULE_HOME=${PROJECT_SOURCE_DIR}
    
)

# ================================================================================================================================== #
