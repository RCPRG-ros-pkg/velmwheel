# ====================================================================================================================================
# @file       ament.cmake
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Tuesday, 24th May 2022 6:18:52 pm
# @modified   Wednesday, 25th May 2022 2:25:08 pm
# @project    engineering-thesis
# @brief      Definition of helper ament-related entities
#    
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# Check if COLCON_PREFIX_PATH variable is set
if(NOT ENV{COLCON_PREFIX_PATH})
    
    # ----------------------------------------------------------------------------------------
    # If path is not set, it means that no package has been built yet in a way that made
    # user able to source `install/setup.bash` file. But as the `cmake_common` package itself
    # must have been built (as the CMake is processing current code), we can obtain the
    # COLCON_PREFIX_PATH variable in a 'hacky' way based on the installation dir of the
    # package.
    # ----------------------------------------------------------------------------------------
    set(ENV{COLCON_PREFIX_PATH} "${cmake_common_DIR}/../../../..")

endif()

# Path to the colcon build dir
set(COLCON_LOG_DIR $ENV{COLCON_PREFIX_PATH}/../log)
