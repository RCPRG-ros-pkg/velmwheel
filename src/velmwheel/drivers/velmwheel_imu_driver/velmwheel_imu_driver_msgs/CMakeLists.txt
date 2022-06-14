# ====================================================================================================================================
# @file       CMakeLists.txt
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Tuesday, 1st March 2022 4:30:53 pm
# @modified   Monday, 30th May 2022 7:27:18 pm
# @project    engineering-thesis
# @brief      Minimal CMake file for the 'velmwheel_imu_driver_msgs' package
#    
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Project ============================================================ #

# CMake version
cmake_minimum_required(VERSION 3.8)
# Project header
project(velmwheel_imu_driver_msgs)

# ========================================================== Configuration ========================================================= #

# Add extra warnings
add_compile_options(-Wall -Wextra -Wpedantic)

# Configure C++17 (instead of original C++14) to meet rclcpp requirement for std::variant
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# ========================================================== Dependencies ========================================================== #

# Ament dependency (standard buildtool dependencies)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# Interfaces dependencies
find_package(builtin_interfaces REQUIRED)

# ============================================================ Messages ============================================================ #

# Add message interfaces to the project
rosidl_generate_interfaces(
    
    # Sources
    ${PROJECT_NAME}

        # Messages
        "msg/AccelerationBiasOffsets.msg"
        "msg/GyroBiasOffsets.msg"
        "msg/GyroTemperatures.msg"

        # Bias configuration services
        "srv/bias/GetAccelerationBiasOffsets.srv"
        "srv/bias/SetAccelerationBiasOffsets.srv"
        "srv/bias/GetBiasOffsets.srv"
        "srv/bias/SetBiasOffsets.srv"
        "srv/bias/GetGyroBiasOffsets.srv"
        "srv/bias/SetGyroBiasOffsets.srv"
        # Digital filter configruation services
        "srv/filter/GetDigitalFilter.srv"
        "srv/filter/SetDigitalFilter.srv"
        # Gyro range configruation services
        "srv/range/GetGyroRange.srv"
        "srv/range/SetGyroRange.srv"
        # Other services
        "srv/CalibrateGyroBias.srv"
        "srv/CalibratePreciseGyroBias.srv"
        "srv/GetGyroTemperatures.srv"
        "srv/GetProductInfo.srv"
        "srv/RestoreFactoryCalibration.srv"

    # Dependencies
    DEPENDENCIES
        builtin_interfaces
)

# Export ament dependencies for generated interfaces
ament_export_dependencies(rosidl_default_runtime)

# ============================================================== Build ============================================================= #

# Build package using ament
ament_package()
