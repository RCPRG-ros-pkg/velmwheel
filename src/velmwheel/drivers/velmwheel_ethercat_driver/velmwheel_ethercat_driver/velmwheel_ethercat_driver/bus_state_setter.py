#!/usr/bin/env python3
# ====================================================================================================================================
# @file       bus_state_setter.py
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Thursday, 28th April 2022 1:05:25 pm
# @modified   Friday, 1st July 2022 5:45:29 pm
# @project    engineering-thesis
# @brief      Auxiliary ROS2 CLI utility requesting the running EtherCAT driver node for setting bus state
# @details    Usage:
#
#      ros2 run velmwheel_ethercat_driver bus_state_setter                           \
#          <fully_qualified_driver_node_name>                                        \
#             --ros-args                                                             \
#             -p bus_state:=<'stopped' or 'running'>                                 \
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
from velmwheel_ethercat_driver_msgs.srv import SetBusState
from velmwheel_ethercat_driver.common import SET_BUS_STATE_SERVICE_NAME
from package_common_py.requester import Requester

# ============================================================== Nodes ============================================================= #

class BusStateSetter():

    """Node class implementing bus-state-set request"""

    def create_request(self, node):

        """Creates service request"""

        # Register 'bus_state' parameter
        node.declare_parameter(
            name='bus_state',
            value=rclpy.Parameter.Type.STRING,
            descriptor=ParameterDescriptor(description="Bus state to be set (either 'stopped' or 'running')"))

        # Parse parameters
        bus_state = node.get_parameter('bus_state').get_parameter_value().string_value

        # Create request
        req = SetBusState.Request()
        # Fill request
        if bus_state == 'stopped':
            req.state = SetBusState.Request.STOPPED
        elif bus_state == 'running':
            req.state = SetBusState.Request.RUNNING
        else:
            raise InvalidParameterValueException('bus_state', bus_state, "Parameter value must be either 'stopped' or 'running'")

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
        srv_type    = SetBusState,
        topic_name  = f'{sys.argv[1]}/{SET_BUS_STATE_SERVICE_NAME}',
        srv_handler =  BusStateSetter()
    )

    # Run request
    requester.run()

# ================================================================================================================================== #

if __name__ == '__main__':
    main()
