# Description

This directory contains port of some of packages from [scan_tools](https://github.com/ccny-ros-pkg/scan_tools) package into the ROS2 world.

# Note 

The original implementation of the `laser_scan_matcher` suffers from invalid TF transformations translating ICP results (as well as initial guess
provided from IMU/Odometry/Velocity model) between fraemes of interest (refernce, keyframe, base and laser). This implementation aims to fix
these bugs as well as provided documentation for these fixes.
