#!/usr/bin/env python3
# ====================================================================================================================================
# @file       drivers_lister.py
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Thursday, 28th April 2022 1:05:25 pm
# @modified   Tuesday, 14th June 2022 3:17:58 pm
# @project    engineering-thesis
# @brief      Auxiliary ROS2 CLI utility lising drivers currently loaded to the running EtherCAT driver node
# @details    Usage
#
#      ros2 run velmwheel_ethercat_driver drivers_lister                             \
#          <fully_qualified_driver_node_name>                                        \
#             --ros-param                                                            \
#             -p service_wait_timeout:=<timeout_of_service_registration:default=1.0>
#
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Imports ============================================================ #

# System imports
import sys
from tabulate import tabulate
# Private imports
from velmwheel_ethercat_driver_msgs.srv import ListDrivers
from velmwheel_ethercat_driver.common import LIST_DRIVERS_SERVICE_NAME
from package_common_py.requester import Requester

# ============================================================== Nodes ============================================================= #

class DriversLister():

    """Node class implementing drivers-listing request"""

    def create_request(self, node):

        """Creates service request"""

        return ListDrivers.Request()

    def handle_response(self, node, response):

        """Handles response"""

        # If service failed, print log
        if not response.success:
            node.get_logger().error(f'Failed to list currently loaded slave driver ({response.error_message})')
        # Else, print listed drivers
        else:
            # Print header
            node.get_logger().info(f'Currently loaded drivers:')
            # Print list table
            node.get_logger().info(tabulate(
                # Columns
                zip(response.unique_ids, response.names),
                # Columns names
                ['ID', 'Name'],
                # Format
                tablefmt="grid"
            ))

# ============================================================== Main ============================================================== #

def main():

    # Create requester
    requester = Requester(
        node_name   = 'drivers_lister',
        srv_type    = ListDrivers,
        topic_name  = f'{sys.argv[1]}/{LIST_DRIVERS_SERVICE_NAME}',
        srv_handler =  DriversLister()
    )

    # Run request
    requester.run()

# ================================================================================================================================== #

if __name__ == '__main__':
    main()
