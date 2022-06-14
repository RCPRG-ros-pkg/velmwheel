#!/usr/bin/env python3
# ====================================================================================================================================
# @file       driver_unloader.py
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Thursday, 28th April 2022 1:05:25 pm
# @modified   Tuesday, 14th June 2022 3:17:53 pm
# @project    engineering-thesis
# @brief      Auxiliary ROS2 CLI utility unloading requesed driver plugin from the running EtherCAT driver node
# @details    Usage
#
#      ros2 run velmwheel_ethercat_driver driver_unloader                            \
#          <fully_qualified_driver_node_name>                                        \
#             --ros-param                                                            \
#             -p id:=<unique_id_of_loaded_driver>                                    \
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
# Private imports
from velmwheel_ethercat_driver_msgs.srv import UnloadDriver
from velmwheel_ethercat_driver.common import UNLOAD_DRIVER_SERVICE_NAME
from package_common_py.requester import Requester

# ============================================================== Nodes ============================================================= #

class DriverUnloader():

    """Node class implementing unloading request"""

    def create_request(self, node):

        """Creates service request"""

        # Register 'id' parameter
        node.declare_parameter(
            name='id',
            value=rclpy.Parameter.Type.INTEGER,
            descriptor=ParameterDescriptor(description='Unique ID of the driver to be unloaded'))

        # Parse parameters
        id = node.get_parameter('id').get_parameter_value().string_value

        # Create request
        req = UnloadDriver.Request()
        # Fill request
        req.unique_id = id

        return req

    def handle_response(self, node, response):

        """Handles response"""

        # If service failed, print log
        if not response.success:
            node.get_logger().error(f'Failed to unload [{node.req.unique_id}] slave driver ({response.error_message})')
        # Else, print success message
        else:
            node.get_logger().info(f'Succesfully unloaded [{node.req.unique_id}] slave driver')

# ============================================================== Main ============================================================== #

def main():

    # Create requester
    requester = Requester(
        node_name   = 'driver_unloader',
        srv_type    = UnloadDriver,
        topic_name  = f'{sys.argv[1]}/{UNLOAD_DRIVER_SERVICE_NAME}',
        srv_handler =  DriverUnloader()
    )

    # Run request
    requester.run()

# ================================================================================================================================== #

if __name__ == '__main__':
    main()
