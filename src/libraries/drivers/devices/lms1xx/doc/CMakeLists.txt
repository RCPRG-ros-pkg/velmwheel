# ====================================================================================================================================
# @file       CMakeLists.txt
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Wednesday, 16th March 2022 4:33:51 pm
# @modified   Monday, 4th July 2022 2:26:42 pm
# @project    engineering-thesis
# @brief      CMake file for documentatio of the `lms1xx` package
#    
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ========================================================== Dependencies ========================================================== #

# Dependencies
find_packages(doc_common)

# =============================================================== Docs ============================================================= #

# List Doxygen source files
file(GLOB_RECURSE SOURCE_INCLUDE_FILES LIST_DIRECTORIES false ${PROJECT_SOURCE_DIR}/include/*.hpp ${PROJECT_SOURCE_DIR}/src/*.cpp)
# Compile source lists
set(DOXYGEN_SOURCE_FILES
    ${SOURCE_INCLUDE_FILES}
)

# List Sphinx source files
file(GLOB_RECURSE SPHINX_SOURCE_FILES LIST_DIRECTORIES false ${CMAKE_CURRENT_SOURCE_DIR}/*.rst)

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
    # Source files (Sphinx)
    SOURCE_FILES ${SPHINX_SOURCE_FILES}
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
