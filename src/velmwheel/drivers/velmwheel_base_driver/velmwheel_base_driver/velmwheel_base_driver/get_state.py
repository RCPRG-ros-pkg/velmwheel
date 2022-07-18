#!/usr/bin/env python3
# ====================================================================================================================================
# @file       get_state.py
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Tuesday, 14th June 2022 3:15:37 pm
# @modified   Friday, 1st July 2022 6:08:40 pm
# @project    engineering-thesis
# @brief      Auxiliary ROS2 CLI utility examining state of the base driver plugin
# @details    Usage:
#
#      ros2 run velmwheel_base_driver get_state.py                                   \
#             --ros-args                                                             \
#             -p service_wait_timeout:=<timeout_of_service_registration:default=1.0>
#
#    <state> may be {1, enable, up} to enable the driver or {0, disable, down} to disable it
#
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Imports ============================================================ #

# System imports
import sys
# Private imports
from velmwheel_base_driver_msgs.srv import GetState
from package_common_py.requester import Requester
from velmwheel_model.params import ROBOT_NAME

# ============================================================== Nodes ============================================================= #

class BaseDriverStateGetter():

    """Node class implementing base-state-get request"""

    def create_request(self, node):

        """Creates service request"""
        
        req = GetState.Request()

        return req

    def handle_response(self, node, response):

        """Handles response"""

        # If service failed, print log
        if not response.success:
            node.get_logger().error(f'Failed to get driver state ({response.error_message})')
        # Else, print success message
        else:

            # Stringify state
            if response.state == 0:
                state = 'Inactive'
            elif response.state == 1:
                state = 'Active'
            elif response.state == 2:
                state = 'Fault'
            else:
                state = 'Recovering from fault'

            # Print state
            node.get_logger().info(f'Driver state is: {state}')

# ============================================================== Main ============================================================== #

def main():

    # Create requester
    requester = Requester(
        node_name   = f'base_driver_state_getter',
        srv_type    = GetState,
        topic_name  = f'/{ROBOT_NAME}/base/get_state',
        srv_handler =  BaseDriverStateGetter()
    )

    # Run request
    requester.run()

# ================================================================================================================================== #

if __name__ == '__main__':
    main()
