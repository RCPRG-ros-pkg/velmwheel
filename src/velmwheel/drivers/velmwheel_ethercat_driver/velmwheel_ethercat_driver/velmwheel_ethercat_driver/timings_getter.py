#!/usr/bin/env python3
# ====================================================================================================================================
# @file       timings_getter.py
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Thursday, 28th April 2022 1:05:25 pm
# @modified   Tuesday, 14th June 2022 3:18:16 pm
# @project    engineering-thesis
# @brief      Auxiliary ROS2 CLI utility requesting the running EtherCAT driver node for measuring bus timing parameters
# @details    Usage
#
#      ros2 run velmwheel_ethercat_driver timings_getter                             \
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
from velmwheel_ethercat_driver_msgs.srv import GetTimingInfo
from velmwheel_ethercat_driver.common import GET_BUS_TIMING_SERVICE_NAME
from package_common_py.requester import Requester

# ============================================================== Nodes ============================================================= #

class TimingsGetter():

    """Node class implementing timings-info request"""

    def create_request(self, node):

        """Creates service request"""

        return GetTimingInfo.Request()

    def handle_response(self, node, response):

        """Handles response"""

        # If service failed, print log
        if not response.success:
            node.get_logger().error(f'Failed to obtain bus timings informations ({response.error_message})')
        # Else, print timing parameters
        else:
            node.get_logger().info( f'Obtained timing parameters:'                                       )
            node.get_logger().info( f'  bus_cycle              = {response.bus_cycle} [ns]'              )
            node.get_logger().info( f'  frame_transmition_time = {response.frame_transmition_time} [ns]' )
            node.get_logger().info( f'  expected_bus_delay     = {response.expected_bus_delay} [ns]'     )
            node.get_logger().info( f'  expected_rx_end_time   = {response.expected_rx_end_time} [ns]'   )
            node.get_logger().info( f'  expected_tx_end_time   = {response.expected_tx_end_time} [ns]'   )

# ============================================================== Main ============================================================== #

def main():

    # Create requester
    requester = Requester(
        node_name   = 'timings_getter',
        srv_type    = GetTimingInfo,
        topic_name  = f'{sys.argv[1]}/{GET_BUS_TIMING_SERVICE_NAME}',
        srv_handler =  TimingsGetter()
    )

    # Run request
    requester.run()

# ================================================================================================================================== #

if __name__ == '__main__':
    main()
