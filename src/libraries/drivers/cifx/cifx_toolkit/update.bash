# ====================================================================================================================================
# @file       update.bash
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Tuesday, 13th April 2021 6:27:36 pm
# @modified   Wednesday, 25th May 2022 9:29:13 pm
# @project    engineering-thesis
# @brief      Script used to update package's content with newer version of the Hilscher's CIFX/netX Toolkit
# 
# @warning Current version of the driver MAY contain some hot-fixes to the Toolkit's source codes (search for @modified tag
#    in Toolkit's files). Before updating the Toolkit make a backup of this version of the packet.
# @note After updating the Toolkit's version, changes to the CMakeLists.txt file may be required to build library properly.
# @note If structure of the files inside the Hilscher's Toolkit package change in the future, some modification to the script
#    may be required 
# 
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# Get full path to the packet's directory
PACKAGE_HOME="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"

# ========================================================== Configruation ========================================================= #

# Logs context
LOG_CONTEXT="cifx_toolkit"

# URL to the actual Toolkit's version's package
TOOLKIT_URL='https://kb.hilscher.com/download/attachments/126430138/NXDRV-TKIT_2020-12-1_V2.6.0.0.zip?api=v2'

# ============================================================ Updating ============================================================ #

# Create temporary folder
mkdir /tmp/toolkit

# Download the package
log_info "Downloading Toolkit"
wget $TOOLKIT_URL -O /tmp/toolkit/toolkit.zip

# Extract toolkit
log_info "Unzipping files"
unzip -d /tmp/toolkit /tmp/toolkit/toolkit.zip > /dev/null
rm /tmp/toolkit/toolkit.zip
mv /tmp/toolkit/*/* /tmp/toolkit/

# Move toolkit's files to the package
log_info "Copying files to the package"
if [[ -d /tmp/toolkit/cifXToolkit/Source ]]; then
    rm $PACKAGE_HOME/src/toolkit/*
    mv /tmp/toolkit/cifXToolkit/Source/*.c $PACKAGE_HOME/src/toolkit/
    rm $PACKAGE_HOME/include/toolkit/*
    mv /tmp/toolkit/cifXToolkit/Source/*.h $PACKAGE_HOME/include/toolkit/
fi

# Move CIFX API header files to the package
if [[ -d /tmp/toolkit/cifXToolkit/Common/cifxAPI ]]; then
    rm $PACKAGE_HOME/include/cifx-api/*
    mv /tmp/toolkit/cifXToolkit/Common/cifXAPI/* $PACKAGE_HOME/include/cifx-api/
fi

# Move Hilscher Definitions header files to the package
if [[ -d /tmp/toolkit/cifXToolkit/Common/HilscherDefinitions ]]; then
    rm $PACKAGE_HOME/include/hilscher-definitions/*
    mv /tmp/toolkit/cifXToolkit/Common/HilscherDefinitions/* $PACKAGE_HOME/include/hilscher-definitions/
fi

# Move bootloaders to the package
if [[ -d /tmp/toolkit/cifXToolkit/BSL ]]; then
    rm $PACKAGE_HOME/bootloaders/*
    mv /tmp/toolkit/cifXToolkit/BSL/*.bin $PACKAGE_HOME/bootloaders/
fi

# Move documentation files to the package
if [[ -d /tmp/toolkit/Documentation ]]; then
    rm -rf $PACKAGE_HOME/doc/materials/*
    mkdir $PACKAGE_HOME/doc/materials/dpm
    mv /tmp/toolkit/Documentation/cifX\ netX\ Application*            $PACKAGE_HOME/doc/materials/
    mv /tmp/toolkit/Documentation/cifX\ netX\ Toolkit*                $PACKAGE_HOME/doc/materials/
    mv /tmp/toolkit/Documentation/cifX\ API*                          $PACKAGE_HOME/doc/materials/
    mv /tmp/toolkit/Documentation/netX\ Dual-Port\ Memory\ Interface* $PACKAGE_HOME/doc/materials/
    mv /tmp/toolkit/Documentation/Second*                             $PACKAGE_HOME/doc/materials/
    mv /tmp/toolkit/Documentation/netX\ Dual-Port\ Memory*            $PACKAGE_HOME/doc/materials/dpm
fi

# Remove tmp directory
rm -rf /tmp/toolkit
