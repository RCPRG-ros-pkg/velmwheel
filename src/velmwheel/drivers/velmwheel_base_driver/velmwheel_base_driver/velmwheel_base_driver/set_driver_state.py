#!/usr/bin/env python3
# ====================================================================================================================================
# @file       set_driver_state.py
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Tuesday, 14th June 2022 3:15:37 pm
# @modified   Tuesday, 14th June 2022 3:15:56 pm
# @project    engineering-thesis
# @brief      Auxiliary ROS2 CLI utility requesting state change in the base driver plugin
# @details    Usage:
#
#      ros2 run velmwheel_base_driver set_driver_state                               \
#          <fully_qualified_driver_node_name> <state>                                \
#             --ros-param                                                            \
#             -p service_wait_timeout:=<timeout_of_service_registration:default=1.0>
#
#    <state> may be {1, enable, up} to enable the driver or {0, disable, down} to disable it
#
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Imports ============================================================ #

# System imports
from msilib.schema import Error
import sys
# Private imports
from velmwheel_base_driver_msgs.srv import Command
from package_common_py.requester import Requester

# ============================================================== Nodes ============================================================= #

class DriverStateSetter():

    """Node class implementing bus-state-get request"""

    def create_request(self, node):

        """Creates service request"""
        
        req = Command.Request()

        # Fill request
        if(sys.argv[1] == '1' or sys.argv[1] == 'enable' or sys.argv[1] == 'up'):
            req.enable = True
        elif(sys.argv[1] == '0' or sys.argv[1] == 'disable' or sys.argv[1] == 'down'):
            req.enable = False
        else:
            raise Error(f'Invalid value of <enable> ({sys.argv[1]})')

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
        node_name   = 'base_driver_state_setter',
        srv_type    = Command,
        topic_name  = f'{sys.argv[1]}/base/command',
        srv_handler =  DriverStateSetter()
    )

    # Run request
    requester.run()

# ================================================================================================================================== #

if __name__ == '__main__':
    main()
