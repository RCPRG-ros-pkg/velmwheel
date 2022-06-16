# ====================================================================================================================================
# @file       ros.bash
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Thursday, 2nd January 2021 2:53:45 pm
# @modified   Thursday, 16th June 2022 12:25:50 pm
# @project    engineering-thesis
# @brief      Setups environment and installs ROS2 distribution along with recommended tools
# 
# 
# @note Script was tested on Ubuntu 20.0.4 LTS system
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# Source BashUitils library
source $BASH_UTILS_HOME/source_me.bash

# ========================================================== Configuration ========================================================= #

# Set distro
export ROS2_DISTRO="$PROJECT_ROS_DISTRO"

# ============================================================== Main ============================================================== #

main() {

    # Install ros
    if ! is_pkg_installed "ros-${PROJECT_ROS_DISTRO}-desktop"; then
        $BASH_UTILS_BIN_HOME/install/ros/ros.bash --distro=$PROJECT_ROS_DISTRO install pkg
    fi

    # Install Gazebo package
    if ! is_pkg_installed 'gazebo11'; then
        $BASH_UTILS_BIN_HOME/install/utilities/gazebo.bash
    fi

    # ROS dependencies
    local -a ros_dependencies=(
        
        # Velmwheel utilities
        xterm
        
        # External libraries
        libpcl-dev
        librange-v3-dev

        # ROS packages
        ros-${PROJECT_ROS_DISTRO}-gazebo-ros-pkgs
        ros-${PROJECT_ROS_DISTRO}-joint-state-publisher
        ros-${PROJECT_ROS_DISTRO}-xacro
        ros-${PROJECT_ROS_DISTRO}-slam-toolbox
        ros-${PROJECT_ROS_DISTRO}-pointcloud-to-laserscan
        ros-${PROJECT_ROS_DISTRO}-robot-localization
        ros-${PROJECT_ROS_DISTRO}-pcl-ros
        ros-${PROJECT_ROS_DISTRO}-navigation2
        ros-${PROJECT_ROS_DISTRO}-nav2-bringup

        # Auxiliary ROS packages (mainly for testing)
        ros-${PROJECT_ROS_DISTRO}-turtlebot3
        ros-${PROJECT_ROS_DISTRO}-turtlebot3-bringup
        ros-${PROJECT_ROS_DISTRO}-turtlebot3-cartographer
        ros-${PROJECT_ROS_DISTRO}-turtlebot3-description
        ros-${PROJECT_ROS_DISTRO}-turtlebot3-example
        ros-${PROJECT_ROS_DISTRO}-turtlebot3-fake-node
        ros-${PROJECT_ROS_DISTRO}-turtlebot3-fake-node-dbgsym
        ros-${PROJECT_ROS_DISTRO}-turtlebot3-gazebo
        ros-${PROJECT_ROS_DISTRO}-turtlebot3-gazebo-dbgsym
        ros-${PROJECT_ROS_DISTRO}-turtlebot3-msgs
        ros-${PROJECT_ROS_DISTRO}-turtlebot3-msgs-dbgsym
        ros-${PROJECT_ROS_DISTRO}-turtlebot3-navigation2
        ros-${PROJECT_ROS_DISTRO}-turtlebot3-node
        ros-${PROJECT_ROS_DISTRO}-turtlebot3-node-dbgsym
        ros-${PROJECT_ROS_DISTRO}-turtlebot3-simulations
        ros-${PROJECT_ROS_DISTRO}-turtlebot3-teleop

    )

    # Install ROS dependencies
    install_pkg_list --su -yv ros_dependencies
    # Update dependencies
    rosdep install                                 \
        --rosdistro=${ROS2_DISTRO}                 \
        --from-paths /opt/ros/${ROS2_DISTRO}/share \
        --ignore-src -y
}

# ============================================================= Script ============================================================= #

# Run the script
source $BASH_UTILS_HOME/lib/scripting/templates/base.bash
