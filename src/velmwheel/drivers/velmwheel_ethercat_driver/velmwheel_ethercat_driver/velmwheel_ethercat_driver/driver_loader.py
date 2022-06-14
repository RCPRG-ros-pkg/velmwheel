#!/usr/bin/env python3
# ====================================================================================================================================
# @file       driver_loader.py
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Thursday, 28th April 2022 1:05:25 pm
# @modified   Tuesday, 14th June 2022 3:17:42 pm
# @project    engineering-thesis
# @brief      Auxiliary ROS2 CLI utility loading requesed driver plugin to the running EtherCAT driver node
# @details    Usage
#
#      ros2 run velmwheel_ethercat_driver driver_loader                              \
#          <fully_qualified_driver_node_name>                                        \
#             --ros-param                                                            \
#             -p plugin_name:=<plugin_name>                                          \
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
from velmwheel_ethercat_driver_msgs.srv import LoadDriver
from velmwheel_ethercat_driver.common import LOAD_DRIVER_SERVICE_NAME
from package_common_py.requester import Requester

# ============================================================== Nodes ============================================================= #

class DriverLoader():

    """Node class implementing load-driver request"""

    def create_request(self, node):

        """Creates service request"""

        # Register 'plugin_name' parameter
        node.declare_parameter(
            name='plugin_name',
            value=rclpy.Parameter.Type.STRING,
            descriptor=ParameterDescriptor(description='A plugin within the ROS package "package_name"'))

        # Parse parameters
        plugin_name = node.get_parameter('plugin_name').get_parameter_value().string_value


    def handle_response(self, node, response):

        """Handles response"""

        # If service failed, print log
        if not response.success:
            node.get_logger().error(f'Failed to load [{node.req.plugin_name}] slave driver ({response.error_message})')
        # Else, print ID of the loaded driver
        else:
            node.get_logger().info(f'Succesfully loaded [{node.req.plugin_name}] slave driver (ID: {response.unique_id})')

# ============================================================== Main ============================================================== #

def main():

    # Create requester
    requester = Requester(
        node_name   = 'driver_loader',
        srv_type    = LoadDriver,
        topic_name  = f'{sys.argv[1]}/{LOAD_DRIVER_SERVICE_NAME}',
        srv_handler =  DriverLoader()
    )

    # Run request
    requester.run()

# ================================================================================================================================== #

if __name__ == '__main__':
    main()
