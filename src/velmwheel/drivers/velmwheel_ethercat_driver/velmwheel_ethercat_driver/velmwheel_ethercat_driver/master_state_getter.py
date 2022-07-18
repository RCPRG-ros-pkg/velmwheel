#!/usr/bin/env python3
# ====================================================================================================================================
# @file       master_state_getter.py
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Thursday, 28th April 2022 1:05:25 pm
# @modified   Friday, 1st July 2022 5:45:51 pm
# @project    engineering-thesis
# @brief      Auxiliary ROS2 CLI utility requesting the running EtherCAT driver node for reading master state
# @details    Usage:
#
#      ros2 run velmwheel_ethercat_driver master_state_getter                      \
#          <fully_qualified_driver_node_name>                                      \
#             --ros-args                                                           \
#             -p service_wait_timeout:=<timeout_of_service_registration:default=1.0>
#
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Imports ============================================================ #

# System imports
import sys
# Private imports
from velmwheel_ethercat_driver_msgs.srv import GetMasterState
from velmwheel_ethercat_driver.common import GET_MASTER_STATE_SERVICE_NAME
from package_common_py.requester import Requester

# ======================================================= Auxiliary functions ====================================================== #

def state_to_str(state):
    if state == GetMasterState.Response.INIT:
        return "INIT"
    if state == GetMasterState.Response.PREOP:
        return "PREOP"
    if state == GetMasterState.Response.SAFEOP:
        return "SAFEOP"
    if state == GetMasterState.Response.OP:
        return "OP"
    if state == GetMasterState.Response.BUSOFF:
        return "BUSOFF"
    if state == GetMasterState.Response.LEAVEOP:
        return "LEAVEOP"
    if state == GetMasterState.Response.BUSSCAN:
        return "BUSSCAN"
    if state == GetMasterState.Response.BUSSCANCOMPLETE:
        return "BUSSCANCOMPLETE"

# ============================================================== Nodes ============================================================= #

class BusStateGetter():

    """Node class implementing request"""

    def create_request(self, node):

        """Creates service request"""

        return GetMasterState.Request()

    def handle_response(self, node, response):

        """Handles response"""

        # If service failed, print log
        if not response.success:
            node.get_logger().error(f'Failed to get bus state ({response.error_message})')
        # Else, print bus state
        else:
            current_state = state_to_str(response.current_state)
            target_state  = state_to_str(response.target_state)
            node.get_logger().info( f'Obtained state info:                                                                        ' )
            node.get_logger().info( f'  current_state                          = {current_state}                                  ' )
            node.get_logger().info( f'  target_state                           = {target_state}                                   ' )
            node.get_logger().info( f'  stop_reason                            = {response.stop_reason}                           ' )
            node.get_logger().info( f'  at_least_one_mandatory_slave_not_in_op = {response.at_least_one_mandatory_slave_not_in_op}' )
            node.get_logger().info( f'  dc_xrmw_stopped                        = {response.dc_xrmw_stopped}                       ' )
            node.get_logger().info( f'  at_least_one_mandatory_slave_lost      = {response.at_least_one_mandatory_slave_lost}     ' )

# ============================================================== Main ============================================================== #

def main():

    # Create requester
    requester = Requester(
        node_name   = 'bus_state_getter',
        srv_type    = GetMasterState,
        topic_name  = f'{sys.argv[1]}/{GET_MASTER_STATE_SERVICE_NAME}',
        srv_handler =  BusStateGetter()
    )

    # Run request
    requester.run()

# ================================================================================================================================== #

if __name__ == '__main__':
    main()
