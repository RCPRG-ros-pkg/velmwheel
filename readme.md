# Introduction

This repository provides comprehensive, ROS2-ready development system for *WUT Velmwheel* robot. The project has been based on all 
foregoing projects and theses that have been written for the last few years around the robot. These have been fully redesigned to 
allow transfer of the platform to the second edition of the *Robot Operating System* as well as to meet requirements of the 
*modern C++* design.

The development enviroment consists of:

  - low-level drivers for all devices present in the robot
  - gazebo-11-based simulation of the robot with RVIZ support
  - set of middleware ROS nodes providing basic controls and processing of sensorical data
  - *roslaunch*-based, highly-parametrizable bringup system covering all components of the environment
  - set of *bash* scripts automating system-setup for the project as well as providing some handy tools speeding-up building and bringup process

# Content

The project has been structured in a way that mimics ralationships between individual components of the system. Each source file in the project 
has been well-documented using doxygen/sphinx convention. The following list provides bird-eye view of the project's structure:

  - `extern`: submodule repositories that the project depends on
  - `scripts`: set of *bash* scripts automating system-setup and providing command-line tools speeding-up regular workflow
  - `src`: actual source packages of the project:
    * `libraries`: ROS packages containing C++/Python libraries utilized by the Velmwheel system
    * `nodes`: set of ROS nodes that are not core part of the environment
    * `velmwheel`: core ROS packages implementing main functionalities of the system. This directory has been divided into 5 submodules 
    according to characteristics of their components:
      - `velmwheel/bringup`: packages providing unified *roslaunch*-based bringup system for other packages implemented in rest of submodules; 
      **velmwheel_bringup** provides main *roslaunch* script for the system and should be the only one needed to run the environment
      (for both simulation and real-world scenarios)
      - `velmwheel/common`: common utilities utilized across the system
      - `velmwheel/drivers`: low-level device drivers for the WUT Velmwheel robot
      - `velmwheel/middleware`: basic controls and sensorical-data-processing nodes
      - `velmwheel/sim`: simulation environments for the WUT Velmwheel robot (currently only gazebo-based implementation has been provided)

# Usage

Repository provides system of *bash* utilities automating setup of the development environment based on the open-source 
[bash-utils](https://github.com/kpierczy/bash-utils) library. All of these can be run by:

  - changing `PROJECT_ROS_DISTRO` variable in the `source source_me.bash` file to the target ROS2 version (at the moment project
    has been tested on ROS2 Humble in Ubuntu 22.04LTS environment)
  - typing `source source_me.bash update` into the terminal from the root directory of the project

The latter will inspect the system, install all required dependencies (including target ROS2 distribution), setup key environment
variables and apply patches on system sources as needed. On a fresh system the whole setup process should take up to 20-30 minutes
(there *may* be need for typing down admin password in the meantime). Sucesfully finalized setup will be summarized with 
a green-coloured log message.

When all dependencies are installed, the system is ready to use. All source code is compatible with basic ROS utilities like *colcon* and *roslaunch*.
However recommended way to interact with the project is via set of dedicated *bash* functions. All of them are defined in the `scripts/aliases` directory.
Key utilities among them are:

  - `colbuild_src [--up-to] [packages...]` - builds all/listed packages in the `src/` directory; passing  `--up-to` option will
    force rebuilding of all dependencies of listed packages
  - `colbuild_prj [--up-to] [packages...]` - analogous to `colbuild_src`; includes `extern/ros` packages into the build
  - `coltest_src [packages...]` - runs tests for all/listed packages in the `src/` directory
  - `bringup [...]` - simple wrapper around `ros launch` running main launchfile of the project configured for real-world scenario;
    all arguments passed to this command will be appended to the `ros launch` call
  - `bringup_sim WORLD_NAME [...]` - simple wrapper around `ros launch` running main launchfile of the project configured for
    simulation scenario; `WORLD_NAME` is path to the world file either relative to the `gazebo/media/world` subdirectory of the
    `velmwheel_gazebo` package or an absolute path; all other arguments passed to this command will be appended to the `ros launch`
    call

Last two commands are defined in the `scripts/aliases/launching.bash` file and are used mainly to avoid typing all launch parameters at 
command-line every time system is run. The list of parameters can (and probably should) be adjusted depending on requirements.

All commands are automatically added to the terminal every time the `source_me.bash` file is sourced (it will also source global ROS setup.bash 
as well as setup files of locally built packages if exist). Ommiting `update` keyword will skip dependencies-verification procedure and reduce
sourcing time.

## Launch parameters

The project introduces 55 packages from of which more than 25 constitute core of the Velmwheel environment. Every functional core package defines it's 
dedicated launchfile with set of specific launch parameters. This results in a large set of values that can be adjusted during system bringup. In all
launchfiles, parameters are defined in a self-documenting way at the beggining of the file. General form of the definition is:

    <...>, <...> = declare_launch_argument({
        'name':          'velmwheel_log_level',
        'default_value': 'warn',
        'description':   'Log level of the run nodes',
        'choices': [ 'info', 'warn', 'error' ]
    })

All packages define set of common parameters (see `node.py` file in the `velmwheel_launch` package). These are:

  - `velmwheel_log_level`: system-wide log level configuration for the nodes
  - `with_<component_name>`: boolean parameter deciding whether the given component should be run
  - `<component_name>_required`: boolean parameter deciding whether launch should shutdown when the node exits (default 'false'
    for most of nodes)
  - `<component_name>_config`: path to the ROS configuration file that will be **appended** to the list of configruation
    parameters of the component at launchtime (note that these parameters will be appended, i.e. they can overwite some
    values set by the launchfile by default)

Note that naming convention for `<component_name>` is following: for each `velmwheel_*` package providing `velmwheel_*` node executable/component
value of the `<component_name>` is `*`. E.g. if `velmwheel_ethercat_driver` package provides `velmwheel_ethercat_driver` node executable, the
corresponding launchfile will define `ethercat_driver_required` launch argument. All parameters can be listed with 
`ros2 launch -s <package> <launchfile>` command, although introspection capabilities for nested launchfiles are currently very limited (see 
[discussion](https://github.com/ros2/launch/issues/313)).
