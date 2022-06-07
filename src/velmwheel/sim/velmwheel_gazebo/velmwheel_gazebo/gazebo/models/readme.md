# Introduction

This directory contains SDF modules constituting model of the WUT Velmwheel robot. These has been originally sourced from
the [omnivelma](https://github.com/RCPRG-ros-pkg/omnivelma) repository and changed only a bit to fit requirements of the current 
usage scenario. The following packages from the oroginal repository has been sourced:

- **omnivelma**: main SDF model and the Gazebo plugin
- **omnivelma_msg**: ROS messages used by the Gazebo plugin
- **monokl**: SDF model of the LIDAR sensor
- **wewucho**: SDF model of the IMU sensor
- **ownKinect**: SDF model of the Kinect sensor

# Notes

At the moment, the project has been switched to use the URDF-based robot description. Although the SDF format provides generally 
broader possibilities, there are some arguments behind choosing the native ROS format. For more information see @details tag in
@file launch/components/gazebo.launch.py. The SDF model has been left in the source tree for potential future usage. Hovwever it 
is outdated with respect to the URDF model and should be updated befory being used.
