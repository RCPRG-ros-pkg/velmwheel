#!/usr/bin/env python3
# ====================================================================================================================================
# @file       master_state_setter.py
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Thursday, 28th April 2022 1:05:25 pm
# @modified   Tuesday, 14th June 2022 3:18:09 pm
# @project    engineering-thesis
# @brief      Auxiliary ROS2 CLI utility requesting the running EtherCAT driver node for setting master state
# @details    Usage:
#
#      ros2 run velmwheel_ethercat_driver master_state_setter                        \
#          <fully_qualified_driver_node_name>                                        \
#             --ros-param                                                            \
#             -p master_state:=<state>                                               \
#             -p service_wait_timeout:=<timeout_of_service_registration:default=1.0>
#
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Imports ============================================================ #

# System imports
import sys
# ROS imports
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.exceptions import InvalidParameterValueException
# Private imports
from velmwheel_ethercat_driver_msgs.srv import SetMasterState
from velmwheel_ethercat_driver.common import SET_MASTER_STATE_SERVICE_NAME
from package_common_py.requester import Requester

# ============================================================== Nodes ============================================================= #

class BusStateSetter():

    """Node class implementing bus-state-set request"""

    def create_request(self, node):

        """Creates service request"""

        # Register 'bus_state' parameter
        node.declare_parameter(
            name='master_state',
            value=rclpy.Parameter.Type.STRING,
            descriptor=ParameterDescriptor(description="Bus state to be set (one of 'init', 'preop', 'safeop', 'op')"))

        # Parse parameters
        master_state = node.get_parameter('master_state').get_parameter_value().string_value

        # Create request
        req = SetMasterState.Request()
        # Fill request
        if master_state == 'init':
            req.state = SetMasterState.Request.INIT
        elif master_state == 'preop':
            req.state = SetMasterState.Request.PREOP
        elif master_state == 'safeop':
            req.state = SetMasterState.Request.SAFEOP
        elif master_state == 'op':
            req.state = SetMasterState.Request.OP
        else:
            raise InvalidParameterValueException(
                'master_state',
                master_state,
                "Parameter value must be one of 'init', 'preop', 'safeop', 'op'"
            )

        return req

    def handle_response(self, node, response):

        """Handles response"""

        # If service failed, print log
        if not response.success:
            node.get_logger().error(f'Failed to set bus state ({response.error_message})')
        # Else, print success message
        else:
            node.get_logger().info( f'Bus state sucesfully set')

# ============================================================== Main ============================================================== #

def main():

    # Create requester
    requester = Requester(
        node_name   = 'bus_state_setter',
        srv_type    = SetMasterState,
        topic_name  = f'{sys.argv[1]}/{SET_MASTER_STATE_SERVICE_NAME}',
        srv_handler =  BusStateSetter()
    )

    # Run request
    requester.run()

# ================================================================================================================================== #

if __name__ == '__main__':
    main()
