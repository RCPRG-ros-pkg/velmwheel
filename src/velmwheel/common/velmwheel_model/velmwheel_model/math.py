# ====================================================================================================================================
# @file       math.py
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Friday, 18th March 2022 1:17:35 pm
# @modified   Wednesday, 25th May 2022 11:37:08 pm
# @project    engineering-thesis
# @brief      Routines providing abstract mathematical models related to the WUT Velmwheel robot
#
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Imports ============================================================ #

# Standard messages
from geometry_msgs.msg import Twist
# Velmwheel-specific
from velmwheel_msgs.msg import Wheels
from velmwheel_model.dimensions import *

# ============================================================== Math ============================================================== #

def twist_to_wheels(velocity : Twist) -> Wheels:
    
    """Converts velocity of the WUT Velmwheel robot into the velocity of it's wheels

    Parameters
    ----------
    velocity : geometry_msgs.msg.Twist
        velocity of the robot

    Returns
    -------
    wheel velocities : velmwheel_msgs.msg.Wheels
        velocities of robot's wheels

    Note
    ----
    Calculations based on [4]
    
    Note
    ----
    [1] and [3] provide opposite twist of rollers and so equations must be 'reversed'
    
    Note
    ----
    In [1] Fig. 8 provides wrong visualisation of the wheels' rollers (there is the same
    setup as in Velmwheel, but directions are given for the opposite setup)
    
    Note
    ----
    [4] presents setup identical to velmwheel
    
    See Also
    --------
    [1] https://www.researchgate.net/publication/269368896_Structures_of_the_Omnidirectional_Robots_with_Swedish_Wheels
    [2] https://journals.sagepub.com/doi/abs/10.1177/0954406219843568
    [3] https://core.ac.uk/download/pdf/48633343.pdf
    [4] https://research.ijcaonline.org/volume113/number3/pxc3901586.pdf

    """

    wheels_velocities = Wheels()

    # Calculate coefficient relating angular velocity with the contribution to the wheel's speeds
    ROTATION_COEFFICIENT = (ROBOT_LENGTH_M + ROBOT_WIDTH_M) / 2.0

    # Calculate rotation's contribution to wheels' speeds
    rotation_contribution = velocity.angular.z * ROTATION_COEFFICIENT
    # Calculate velocities for all wheels
    wheels_velocities.fl = (velocity.linear.x - velocity.linear.y - rotation_contribution) / WHEEL_RADIUS_M;
    wheels_velocities.fr = (velocity.linear.x + velocity.linear.y + rotation_contribution) / WHEEL_RADIUS_M;
    wheels_velocities.rl = (velocity.linear.x + velocity.linear.y - rotation_contribution) / WHEEL_RADIUS_M;
    wheels_velocities.rr = (velocity.linear.x - velocity.linear.y + rotation_contribution) / WHEEL_RADIUS_M;

    return wheels_velocities


def wheels_to_twist(wheels_velocities : Wheels) -> Twist:
    
    """Converts velocities of the WUT Velmwheel robot's wheels into the robot's velocity

    Parameters
    ----------
    wheels_velocities : velmwheel_msgs.msg.Wheels
        velocities of robot's wheels

    Returns
    -------
    velocity : geometry_msgs.msg.Twist
        velocity of the robot

    Note
    ----
    Calculations based on [4]
    
    Note
    ----
    [1] and [3] provide opposite twist of rollers and so equations must be 'reversed'
    
    Note
    ----
    In [1] Fig. 8 provides wrong visualisation of the wheels' rollers (there is the same
    setup as in Velmwheel, but directions are given for the opposite setup)
    
    Note
    ----
    [4] presents setup identical to velmwheel
    
    See Also
    --------
    [1] https://www.researchgate.net/publication/269368896_Structures_of_the_Omnidirectional_Robots_with_Swedish_Wheels
    [2] https://journals.sagepub.com/doi/abs/10.1177/0954406219843568
    [3] https://core.ac.uk/download/pdf/48633343.pdf
    [4] https://research.ijcaonline.org/volume113/number3/pxc3901586.pdf

    """

    velocity = Twist

    # Calculate quarter of wheel's radius
    QUARTER_OF_WHEEL_RADIUS = WHEEL_RADIUS_M / 4.0
    # Calculate coefficient relating wheels' speeds with the contribution to the angular velocity of the robot
    ROTATION_COEFFICIENT = 2.0 / (ROBOT_LENGTH_M + ROBOT_WIDTH_M)

    w = wheels_velocities

    # Calculate robot's velocity
    velocity.linear.x  = ( w.fl + w.fr + w.rl + w.rr) * QUARTER_OF_WHEEL_RADIUS
    velocity.linear.y  = (-w.fl + w.fr + w.rl - w.rr) * QUARTER_OF_WHEEL_RADIUS
    velocity.angular.z = (-w.fl + w.fr - w.rl + w.rr) * QUARTER_OF_WHEEL_RADIUS * ROTATION_COEFFICIENT

    return velocity
