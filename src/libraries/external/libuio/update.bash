# ====================================================================================================================================
#  @ Filename: update.bash
#  @ Author: Krzysztof Pierczyk
#  @ Create Time: 2021-04-19 19:38:10
#  @ Modified time: 2021-04-19 19:38:12
#  @ Description: 
#     
#     Updates source files of the `libuio` library from the original git repository
#     
#  # @ Warning: Current version of the driver MAY contain some hot-fixes to the Toolkit's source codes (search for @modified tag
#     in Toolkit's files). Before updating the Toolkit make a backup of this version of the packet.
# ====================================================================================================================================

# Get full path to the packet's directory
PACKAGE_HOME="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"

# ------------------------------------------------------------ Updating --------------------------------------------------------------

# Remove libuio folder from tmp, if exists
rm -rf /tmp/libuio

# Clone original library
echo "[${log}LOG:libuio${rst}] Downloading Communication solution"
git clone https://github.com/missinglinkelectronics/libuio /tmp/libuio

# Remove current source file
rm -f $PACKAGE_HOME/include/*
rm -f $PACKAGE_HOME/src/*

# Move source files to the package
echo "[${log}LOG:libuio${rst}] Copying files to the package"
mv /tmp/libuio/libuio.h $PACKAGE_HOME/include/
mv /tmp/libuio/libuio_internal.h $PACKAGE_HOME/src/
mv /tmp/libuio/*.c $PACKAGE_HOME/src/

# Delete original repository
rm -rf /tmp/libuio
