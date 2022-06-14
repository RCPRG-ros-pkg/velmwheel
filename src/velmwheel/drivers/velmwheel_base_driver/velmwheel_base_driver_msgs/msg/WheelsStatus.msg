# ====================================================================================================================================
# @file       WheelsStatus.msg
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Monday, 13th June 2022 8:35:04 pm
# @modified   Monday, 13th June 2022 10:44:47 pm
# @project    engineering-thesis
# @brief      Message structure holding current status of servodrivers controlling robot's wheels 
#    
#    
# @note Order of wheels in the vector is compatible with <i>velmwheel_msgs</i> interfaces ( @ref WheelsEnum.msg )
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# Possible status values
uint8 NOT_READY_TO_SWITCH_ON = 0
uint8 SWITCHED_ON_DISABLED   = 1
uint8 READY_TO_SWITCH_ON     = 2
uint8 SWITCH_ON              = 3
uint8 OPERATION_ENABLED      = 4
uint8 QUICK_STOP_ACTIVE      = 5
uint8 FAULT_REACTION_ACITVE  = 6
uint8 FAULT                  = 7
uint8 INVALID_STATE          = 8

# Header informations of the measurements
std_msgs/Header header

# Status of wheels
uint8[4] status
