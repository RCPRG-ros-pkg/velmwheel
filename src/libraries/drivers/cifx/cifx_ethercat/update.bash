# ====================================================================================================================================
# @ Filename: update.bash
# @ Author: Krzysztof Pierczyk
# @ Create Time: 2021-04-20 21:18:39
# @ Modified time: 2021-04-20 21:22:36
# @ Description: 
#    
#    Script used to update ethercat master firmware binaries
#    
# @ Note: If structure of the files inside the Hilscher's Communication Solution package change in the future, some modification 
#     to the script may be required 
# ====================================================================================================================================

# Get full path to the packet's directory
PACKAGE_HOME="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"

# ========================================================== Configruation ========================================================= #

# URL to the actual Communication Solution package
COM_SOLUTION_URL="https://www.hilscher.com/fileadmin/big_data/en-US/Resources/zip/Communication_Solutions_DVD_2021-03-2.zip"

# Firmware location inside package
FIRMWARE_LOCATION="Firmware,_EDS,_Examples,_Webpages/Firmware_&_EDS/COMSOL-ECM V4.5.0.0/Firmware/cifX/cifxecm.nxf"

# ============================================================ Updating ============================================================ #

# Create temporary folder
mkdir /tmp/com_solution

# Download the package
log_info "Downloading Communication solution"
wget $COM_SOLUTION_URL -O /tmp/com_solution/com_solution.zip

# Extract toolkit
log_info "Unzipping files"
unzip -d /tmp/com_solution /tmp/com_solution/com_solution.zip > /dev/null
rm /tmp/com_solution/com_solution.zip

# Move toolkit's files to the package
log_info "Copying files to the package"
rm -f $PACKAGE_HOME/firmware/*.nxf
mv "/tmp/com_solution/$FIRMWARE_LOCATION" $PACKAGE_HOME/firmware/

# Remove tmp directory
rm -rf /tmp/com_solution
