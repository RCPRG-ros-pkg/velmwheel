# ====================================================================================================================================
# @file       requester.py
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Thursday, 28th April 2022 1:05:25 pm
# @modified   Tuesday, 14th June 2022 3:29:50 pm
# @project    engineering-thesis
# @brief      Auxiliary Python class providing common interface for creating single-use ROS2 nodes performing request to the
#             desired ROS service
#
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# =============================================================== Doc ============================================================== #

""" 

.. module:: requester
   :platform: Unix
   :synopsis: Auxiliary Python class providing common interface for creating single-use ROS2 nodes performing request to the
              desired ROS service

.. moduleauthor:: Krzysztof Pierczyk <krzysztof.pierczyk@gmail.com>

"""

# ============================================================= Imports ============================================================ #

# ROS imports
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

# ========================================================== RequesterNode ========================================================= #

class RequesterNode(Node):

    """Node class utilized by Requester class to perform ROS service request"""

    def __init__(self, node_name, srv_type, topic_name, srv_handler):

        """Initializes the node and prepares service request
        
        Parameters
        ----------
        node_name : str
            name of the node
        srv_type : str
            target service type
        topic_name
            name of the service topic
        srv_handler : Any
            obejct implementing the following methods:
              - create_request(node): creates request for the service; as an argument
                  takes reference to the node that can be used e.g. to parse ROS parameters
                  specific for the implementation
              - handle_response(response): handle sucesfully response; called by the 
                  Requester if service response has been sucesfully obtained by the 
                  requester node
        """

        # Initialize the node
        super().__init__(node_name)

        # ------------------------------ Parse parameters -------------------------------

        # Register 'service_wait_timeout' parameter
        self.declare_parameter(
            name='service_wait_timeout',
            value=1.0,
            descriptor=ParameterDescriptor(description='Timeout of the loading-service appearance in [s]'))

        # Parse parameters
        service_wait_timeout_s = self.get_parameter('service_wait_timeout').get_parameter_value().double_value

        # --------------------------- Create service client -----------------------------

        # Create service client
        self.cli = self.create_client(srv_type, topic_name)
        # Wait for service to be present on the network
        while not self.cli.wait_for_service(timeout_sec=service_wait_timeout_s):
            raise Exception(f'\'{topic_name}\' service not available')

        # Create request
        self.req = srv_handler.create_request(self)

    def send_request(self):

        """Sens service request prepared in the constructor"""
        
        self.future = self.cli.call_async(self.req)

# ============================================================ Requester =========================================================== #

# ---------------------------------------------------------------------------------------
# @brief Auxiliary Python class providing common interface for creating single-use ROS2 
#   nodes performing request to the desired ROS service
# @details Requester class provides a single method - run() - that handles 
#    request-response using parameters given at construction.
# ---------------------------------------------------------------------------------------
class Requester:

    """Auxiliary Python class providing common interface for creating single-use ROS2 
    nodes performing request to the desired ROS service

    Requester class provides a single method - run() - that handles request-response using
    parameters given at construction.
    """

    def __init__(self, node_name, srv_type, topic_name, srv_handler):

        """Initializes the Requester object
        
        Parameters
        ----------
        node_name : str
            name of the node
        srv_type : str
            target service type
        topic_name
            name of the service topic
        srv_handler : Any
            obejct implementing the following methods:
            
            * **create_request(node)**: creates request for the service; as an argument
              takes reference to the node that can be used e.g. to parse ROS parameters
              specific for the implementation
            * **handle_response(node, response)**: handles sucesfully response; called by the 
              Requester if service response has been sucesfully obtained by the 
              requester node
            
        """


        self.node_name = node_name
        self.srv_type = srv_type
        self.topic_name = topic_name
        self.srv_handler = srv_handler

    def run(self):

        """Runs request-response chain with parameter given at construction"""
        
        # Initialize rlc
        rclpy.init()
        
        # Create node
        try:
            node = RequesterNode(
                self.node_name,
                self.srv_type,
                self.topic_name,
                self.srv_handler
            )
        except Exception as e:
            rclpy.logging.get_logger(f'Requester[{self.node_name}]').error('%r' % (e,))
            rclpy.shutdown()
            exit(1)
        # Send service request
        node.send_request()

        # Wait for response
        while rclpy.ok():

            # Process client ndoe
            rclpy.spin_once(node)
            # If response has been received
            if node.future.done():

                # Try to read the result
                try:
                    response = node.future.result()

                # If service could NOT be handled, print log
                except Exception as e:
                    node.get_logger().info('Service call failed %r' % (e,))

                # If service could be handled, handle response
                else:
                    self.srv_handler.handle_response(node, response)

                # Break the waiting loop
                break

        # Explicitly destroy the node
        node.destroy_node()
        # Shutdown rlc
        rclpy.shutdown()
        
# ================================================================================================================================== #
