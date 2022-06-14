# ====================================================================================================================================
# @file       CMakeLists.txt
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Wednesday, 16th March 2022 4:33:51 pm
# @modified   Thursday, 9th June 2022 4:45:22 am
# @project    engineering-thesis
# @brief      CMake file for `node_common_cpp` package
#    
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ========================================================== Dependencies ========================================================== #

# Dependencies
find_packages(doc_common)

# =============================================================== Docs ============================================================= #

# List Doxygen source files
file(GLOB_RECURSE DOXYGEN_SOURCE_FILES LIST_DIRECTORIES false ${PROJECT_SOURCE_DIR}/include/*.hpp)

# Add documentation target (Doxygen)
add_doxygen_doc(doxygen_doc

    # Working directory
    WORKING_DIR ${PROJECT_SOURCE_DIR}
    # Soruce files
    SOURCE_FILES ${DOXYGEN_SOURCE_FILES}
    # Tag file
    TAG_FILE ${PROJECT_NAME}.tag
    # Install dircetory
    INSTALL_DIR "share/${PROJECT_NAME}/doc/doxygen"
    # Doxyfiles
    DOXY_FILES ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile

)

# Add documentation target (Sphinx)
add_sphinx_doc(sphinx_doc

    # Working directory (Sphinx)
    WORKING_DIR ${PROJECT_SOURCE_DIR}
    # Install dircetory (Sphinx)
    INSTALL_DIR "share/${PROJECT_NAME}/doc/sphinx"

    # Generate C++ API doc using Doxygen & Breathe tools
    GENERATE_CPP_API_DOC

    # Soruce files (Doxygen)
    DOXYGEN_SOURCE_FILES ${DOXYGEN_SOURCE_FILES}
    # Install dircetory (Doxygen)
    DOXYGEN_INSTALL_DIR "share/${PROJECT_NAME}/doc/sphinx-doxygen"
    # Doxygen config files (Doxygen)
    DOXYGEN_DOXY_FILES ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile
    # Doxygen config overwrites (Doxygen)
    DOXYGEN_DOXY_LINES

        # Don't generate HTML & LaTeX output in the Sphinx run
        "GENERATE_HTML  = NO"
        "GENERATE_LATEX = NO"
    
)

# ================================================================================================================================== #
