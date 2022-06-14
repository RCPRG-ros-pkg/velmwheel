# ====================================================================================================================================
# @file       dependencies.bash
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Friday, 29th April 2022 1:22:24 pm
# @modified   Monday, 6th June 2022 8:01:16 pm
# @project    engineering-thesis
# @brief
#    
#    Installs system dependencies of the project
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# Source BashUitils library
source $BASH_UTILS_HOME/source_me.bash

# ============================================================== Main ============================================================== #

main() {

    # System dependencies
    local -a dependencies=(
        
        # Libraries
        libboost-dev
        libc6-dev-i386

        # Python
        python3
        python3-pip
        
        # RT measurements
        rt-tests
        # Images creation
        gnuplot-qt
        
        # Documentation
        doxygen
        plantuml
        
    )

    # Install dependencies
    install_pkg_list --su -yv dependencies
    
    # Python dependencies
    local -a python_dependencies=(
        
        # Documentation
        sphinx
        sphinx-rtd-theme
        sphinx-autodocgen
        sphinxcontrib-napoleon
        breathe

    )

    # Install Python dependencies
    pip_install_list -vU python_dependencies

}

# ============================================================= Script ============================================================= #

# Run the script
source $BASH_UTILS_HOME/lib/scripting/templates/base.bash