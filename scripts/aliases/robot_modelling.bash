# ====================================================================================================================================
# @file     robot_modelling.bash
# @author   Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date     Tuesday, 1st March 2022 4:11:56 pm
# @modified   Thursday, 7th April 2022 5:37:42 pm
# @project  engineering-thesis
# @brief
#    
#    Set of handy function and aliases used when dealing with the project
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ---------------------------------------------------------------------------------------
# @brief Parses URDF.xacro model of the robot with xacro
# ---------------------------------------------------------------------------------------
function xacro_urdf_model() {

    # Arguments
    local out="${1:-./velmehweel.urdf}"

    # Parse the file
    ros2 run xacro xacro $PROJECT_HOME/src/velmwheel/velmwheel_gazebo/urdf/velmwheel.urdf.xacro > "$out"
}

# ---------------------------------------------------------------------------------------
# @brief Parses URDF model of the robot with URDF-SDF parser
# ---------------------------------------------------------------------------------------
function sdf_urdf_model() {

    # Arguments
    local out="${1:-./velmehweel.sdf}"
    local in="${2:-./velmehweel.urdf}"

    # Parse the file
    gz sdf -p "$in" > "$out"
}
