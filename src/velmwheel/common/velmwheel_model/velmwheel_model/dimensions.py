# ====================================================================================================================================
# @file       dimensions.py
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Friday, 18th March 2022 1:18:14 pm
# @modified   Wednesday, 25th May 2022 11:32:28 pm
# @project    engineering-thesis
# @brief      Python constants describing characteristic dimensions of the WUT Velmwheel robot
#    
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# Radius of the wheel in [m]
WHEEL_RADIUS_M = 0.1027
# Radius of the wheel's inner part in [m]
WHEEL_INNER_RADIUS_M = 0.0741
# Radius of the wheel's roller in [m]
WHEEL_ROLLER_RADIUS_M = WHEEL_RADIUS_M - WHEEL_INNER_RADIUS_M
# Robot's with in [m] (from measured between oposite wheels' centres)
ROBOT_WIDTH_M = 0.76
# Robot's length in [m] (from measured between pair wheels' centres)
ROBOT_LENGTH_M = 0.728
