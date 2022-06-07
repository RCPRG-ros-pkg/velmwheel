# ====================================================================================================================================
# @file     controls.bash
# @author   Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date     Tuesday, 1st March 2022 4:11:56 pm
# @modified   Thursday, 28th April 2022 9:20:15 pm
# @project  engineering-thesis
# @brief
#    
#    Set of handy function and aliases used when dealing with the project
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ---------------------------------------------------------------------------------------
# @brief Publishes controlls of Wheels on the /velmwheel/base/controls topic
#    according to the given format string
# @param controls
#    YAML-format controls t be pubblished
# ---------------------------------------------------------------------------------------
function pub_wheel_controls() {
    
    # Source install directory
    source $PROJECT_HOME/install/setup.bash
    # Public controls
    ros2 topic pub --once /velmwheel/base/controls velmwheel_msgs/msg/Wheels "$@"
}

# ---------------------------------------------------------------------------------------
# @brief Publishes 0-value controlls of Wheels on the /velmwheel/base/controls 
#    topic 
# ---------------------------------------------------------------------------------------
function pub_zero_wheel_controls() {
    pub_wheel_controls '{}'
}

# ---------------------------------------------------------------------------------------
# @brief Publishes controlls of Wheels on the /velmwheel/base/controls topic
#    according to the given format string
# @param lin_speed_x
#    linear speed in X direction [m]
# @param lin_speed_y
#    linear speed in Y direction [m]
# @param ang_speed (default: 0)
#    angular speed in [rev/s]
#
# @note Equateions based on @see
# @see https://research.ijcaonline.org/volume113/number3/pxc3901586.pdf
# ---------------------------------------------------------------------------------------
function pub_wheel_twist() {
    
    # Arguments
    local lin_speed_x="$1"
    local lin_speed_y="$2"
    local ang_speed="${3:-0.0}"

    # Constants
    local WHEEL_RADIUS_M="0.1027";
    local ROBOT_WIDTH_HALF_M="0.38";
    local ROBOT_LENGTH_HALF_M="0.364";
    local PI="3.14159265359";

    # Convert ang speed from [deg/s] to [rad/s]
    ang_speed=$(echo "$ang_speed * 2 * $PI" | bc -l)

    # Calculate speeds for wheels
    local w_fl=$(echo "(  $lin_speed_x - $lin_speed_y - ($ROBOT_WIDTH_HALF_M + $ROBOT_LENGTH_HALF_M) * $ang_speed) / $WHEEL_RADIUS_M" | bc -l)
    local w_fr=$(echo "(  $lin_speed_x + $lin_speed_y + ($ROBOT_WIDTH_HALF_M + $ROBOT_LENGTH_HALF_M) * $ang_speed) / $WHEEL_RADIUS_M" | bc -l)
    local w_rl=$(echo "(  $lin_speed_x + $lin_speed_y - ($ROBOT_WIDTH_HALF_M + $ROBOT_LENGTH_HALF_M) * $ang_speed) / $WHEEL_RADIUS_M" | bc -l)
    local w_rr=$(echo "(  $lin_speed_x - $lin_speed_y + ($ROBOT_WIDTH_HALF_M + $ROBOT_LENGTH_HALF_M) * $ang_speed) / $WHEEL_RADIUS_M" | bc -l)

    # Source install directory
    source $PROJECT_HOME/install/setup.bash
    # Public controls
    ros2 topic pub --once /velmwheel/base/controls velmwheel_msgs/msg/Wheels "{ values: [ $w_rl, $w_rr, $w_fl, $w_fr ] }"
}

# ---------------------------------------------------------------------------------------
# @brief Publishes controlls of Wheels on the /velmwheel/velocity_setpoint topic 
#    according to the given format string
# @param lin_speed_x
#    linear speed in X direction [m]
# @param lin_speed_y
#    linear speed in Y direction [m]
# @param ang_speed (default: 0)
#    angular speed in [rev/s]
# ---------------------------------------------------------------------------------------
function pub_robot_twist() {
    
    # Arguments
    local lin_speed_x="$1"
    local lin_speed_y="$2"
    local ang_speed="${3:-0.0}"

    # Source install directory
    source $PROJECT_HOME/install/setup.bash

    # Prepare controls
    local controls="{ linear: { x: $lin_speed_x, y: $lin_speed_y }, angular: { z: $ang_speed } }"

    # Public controls
    ros2 topic pub --once /velmwheel/velocity_setpoint geometry_msgs/msg/Twist "$controls"
}

# ---------------------------------------------------------------------------------------
# @brief Publishes controlls of Wheels on the /velmwheel/velocity_setpoint topic 
#    stopping the robot
# ---------------------------------------------------------------------------------------
function pub_zero_twist() {
    pub_robot_twist '0.0' '0.0' '0.0'
}

# ---------------------------------------------------------------------------------------
# @brief Runs 'turtle_teleop_key' node publishing to the /velmwheel/velocity_setpoint  
#    topic using 'teleop.launch.py' launchfile from 'velmwheel_bringup' package
# ---------------------------------------------------------------------------------------
function launch_teleop() {
    ros2 launch velmwheel_bringup teleop.launch.py
}

# ---------------------------------------------------------------------------------------
# @brief Runs 'turtle_teleop_key' node publishing to the /velmwheel/velocity_setpoint  
#    topic
# ---------------------------------------------------------------------------------------
function run_teleop() {
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
        -r __node:=keyboard_teleop                                  \
        -r __ns:=/velmwheel                                         \
        -r cmd_vel:=/velmwheel/base/velocity_setpoint
}
