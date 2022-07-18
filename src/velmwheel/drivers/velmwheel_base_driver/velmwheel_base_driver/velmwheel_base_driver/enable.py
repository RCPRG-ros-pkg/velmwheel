#!/usr/bin/env python3
# ====================================================================================================================================
# @file       enable.py
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Tuesday, 14th June 2022 3:15:37 pm
# @modified   Friday, 1st July 2022 5:44:59 pm
# @project    engineering-thesis
# @brief      Auxiliary ROS2 CLI utility requesting state change in the base driver plugin
# @details    Usage:
#
#      ros2 run velmwheel_base_driver enable.py                                      \
#          <state>                                                                   \
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
from velmwheel_base_driver_msgs.srv import Enable
from package_common_py.requester import Requester
from velmwheel_model.params import ROBOT_NAME

# ============================================================== Nodes ============================================================= #

class DriverStateSetter():

    """Node class implementing bus-state-get request"""

    def create_request(self, node):

        """Creates service request"""
        
        req = Enable.Request()

        # Fill request
        if(sys.argv[1] == '1' or sys.argv[1] == 'enable' or sys.argv[1] == 'up'):
            req.enable = True
        elif(sys.argv[1] == '0' or sys.argv[1] == 'disable' or sys.argv[1] == 'down'):
            req.enable = False
        else:
            raise ValueError(f'Invalid value of <enable> ({sys.argv[1]})')

        return req

    def handle_response(self, node, response):

        """Handles response"""

        # If service failed, print log
        if not response.success:
            node.get_logger().error(f'Failed to set driver state ({response.error_message})')
        # Else, print success message
        else:
            node.get_logger().info(f'Succesfully set driver state to \'{sys.argv[1]}\'')

# ============================================================== Main ============================================================== #

def main():

    # Create requester
    requester = Requester(
        node_name   = f'base_driver_enabler',
        srv_type    = Enable,
        topic_name  = f'/{ROBOT_NAME}/base/enable',
        srv_handler =  DriverStateSetter()
    )

    # Run request
    requester.run()

# ================================================================================================================================== #

if __name__ == '__main__':
    main()
