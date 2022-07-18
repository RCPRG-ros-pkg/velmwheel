#!/usr/bin/env python3
# ====================================================================================================================================
# @file       reset_fault.py
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Tuesday, 14th June 2022 3:15:50 pm
# @modified   Friday, 1st July 2022 5:48:50 pm
# @project    engineering-thesis
# @brief      Auxiliary ROS2 CLI utility requesting state change in the base driver plugin
# @details    Usage:
#
#      ros2 run velmwheel_base_driver reset_fault                                    \
#          <fully_qualified_driver_node_name>                                        \
#             --ros-param                                                            \
#             -p service_wait_timeout:=<timeout_of_service_registration:default=1.0>
#
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Imports ============================================================ #

# System imports
import sys
# Private imports
from velmwheel_base_driver_msgs.srv import ResetFailure
from package_common_py.requester import Requester
from velmwheel_model.params import ROBOT_NAME

# ============================================================== Nodes ============================================================= #

class DriverFailureResetter():

    """Node class implementing bus-state-get request"""

    def create_request(self, node):

        """Creates service request"""

        return ResetFailure.Request()

    def handle_response(self, node, response):

        """Handles response"""

        # If service failed, print log
        if not response.success:
            node.get_logger().error(f'Failed to reset failure ({response.error_message})')
        # Else, print success message
        else:
            node.get_logger().info(f'Succesfully reset driver\'s failure')

# ============================================================== Main ============================================================== #

def main():

    # Create requester
    requester = Requester(
        node_name   = f'base_driver_failure_resetter',
        srv_type    = ResetFailure,
        topic_name  = f'/{ROBOT_NAME}/base/reset_fault',
        srv_handler =  DriverFailureResetter()
    )

    # Run request
    requester.run()

# ================================================================================================================================== #

if __name__ == '__main__':
    main()
