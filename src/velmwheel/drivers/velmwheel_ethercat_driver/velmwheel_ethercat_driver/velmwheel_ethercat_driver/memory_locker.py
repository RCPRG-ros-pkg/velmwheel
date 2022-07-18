#!/usr/bin/env python3
# ====================================================================================================================================
# @file       memory_locker.py
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Thursday, 28th April 2022 1:05:25 pm
# @modified   Friday, 1st July 2022 5:46:07 pm
# @project    engineering-thesis
# @brief      Auxiliary ROS2 CLI utility requesting the running EtherCAT driver node for locking process memory
# @details    Usage
#
#      ros2 run velmwheel_ethercat_driver memory_locker                            \
#          <fully_qualified_driver_node_name>                                      \
#             --ros-args                                                           \
#             -p memory_lock_scheme:=<lock_scheme>                                 \
#             -p disable_memory_trimming:=<bool>                                   \
#             -p disable_mmap:=<bool>                                              \
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
from velmwheel_ethercat_driver_msgs.srv import LockMemory
from velmwheel_ethercat_driver.common import LOCK_MEMORY_SERVICE_NAME
from package_common_py.requester import Requester

# ============================================================== Nodes ============================================================= #

class MemoryLocker():

    """Node class implementing lock-memory request"""

    def create_request(self, node):

        """Creates service request"""

        # Register 'memory_lock_scheme' parameter
        node.declare_parameter(
            name='memory_lock_scheme',
            value=rclpy.Parameter.Type.STRING,
            descriptor=ParameterDescriptor(description="Memory lock scheme (one of: {'current', 'all'})"))
        # Register 'disable_memory_trimming' parameter
        node.declare_parameter(
            name='disable_memory_trimming',
            value=rclpy.Parameter.Type.BOOL,
            descriptor=ParameterDescriptor(description="If 'true' node will request disabling memory trimming with sbrk()"))
        # Register 'disable_mmap' parameter
        node.declare_parameter(
            name='disable_mmap',
            value=rclpy.Parameter.Type.BOOL,
            descriptor=ParameterDescriptor(description="If 'true' node will request disabling mmap()"))


        # Parse parameters
        memory_lock_scheme      = node.get_parameter('memory_lock_scheme').get_parameter_value().string_value
        disable_memory_trimming = node.get_parameter('disable_memory_trimming').get_parameter_value().bool_value
        disable_mmap            = node.get_parameter('disable_mmap').get_parameter_value().bool_value

        # Create request
        req = LockMemory.Request()
        # Fill request
        if memory_lock_scheme == 'current':
            req.locking_scheme = LockMemory.CURRENT
        elif memory_lock_scheme == 'all':
            req.locking_scheme = LockMemory.ALL
        else:
            raise InvalidParameterValueException('memory_lock_scheme', memory_lock_scheme, "Parameter value must be either 'current' or 'all'")
        req.disable_memory_trimming = disable_memory_trimming
        req.disable_mmap            = disable_mmap

        return req
        
    def handle_response(self, node, response):

        """Handles response"""

        # If service failed, print log
        if not response.success:
            node.get_logger().error(f'Failed to get bus state ({response.error_message})')
        # Else, print success message
        else:
            node.get_logger().info(f'Memory has been succesfully locked')

# ============================================================== Main ============================================================== #

def main():

    # Create requester
    requester = Requester(
        node_name   = 'memory_locker',
        srv_type    = LockMemory,
        topic_name  = f'{sys.argv[1]}/{LOCK_MEMORY_SERVICE_NAME}',
        srv_handler =  MemoryLocker()
    )

    # Run request
    requester.run()

# ================================================================================================================================== #

if __name__ == '__main__':
    main()
