# ====================================================================================================================================
# @file       regex_forwarder.py
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Wednesday, 2nd March 2022 12:19:38 pm
# @modified   Wednesday, 25th May 2022 11:04:46 pm
# @project    engineering-thesis
# @brief      Utility node enabling echoing string content of the ROS topic to another topic with regex replacements applied
#    
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# =============================================================== Doc ============================================================== #

""" 

.. module:: regex_forwarder
   :platform: Unix
   :synopsis: Utility node enabling echoing string content of the ROS topic to another topic with regex replacements applied

.. moduleauthor:: Krzysztof Pierczyk <krzysztof.pierczyk@gmail.com>

"""

# ============================================================= Imports ============================================================ #

# System imports
import re
# ROS imports
from click import Parameter
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rcl_interfaces.msg import ParameterDescriptor 
from rclpy.qos import QoSPolicyKind
from rclpy.qos_overriding_options import QoSOverridingOptions

# ============================================================== Nodes ============================================================= #

class RegexForwarder(Node):

    """Utility node enabling echoing string content of the ROS topic to another topic 
    with regex replacements applied

    Subscribtions:

        * ~/in [std_msgs.msg.String] -- input string

    Publishers:

        * ~/out [std_msgs.msg.String] -- output string

    """

    def __init__(self):

        """Initializes the node with the given name. Creats a publichser object
        associated with the '~/out' topic to publish transformed text into. 
        Subscribes to the ~/in topic that the text is sourced from 
        """

        # Initialize node
        super().__init__('regex_forwarder')

        # Register 'my_parameter' parameter with default value 'world'
        self.declare_parameter(
            name='replacements',
            value=rclpy.Parameter.Type.STRING_ARRAY,
            descriptor=ParameterDescriptor(
                description=
                'List of regex replacement schemes to be applied to each '       +
                'message incoming on the ~/in topic. These are expressions '     +
                'in a form <match>:=<replace>. Form of the <match> tag foolows ' +
                'rules of the Python regex module.'
            )
        )

        # Create subscriber object
        self.sub = self.create_subscription(String, '~/in', self.msg_callback, 10,
            qos_overriding_options=QoSOverridingOptions([QoSPolicyKind.DURABILITY]))
        # Create publisher object
        self.pub = self.create_publisher(String, '~/out', 10,
            qos_overriding_options=QoSOverridingOptions([QoSPolicyKind.DURABILITY]))

    def msg_callback(self, msg):

        """Callback called at new message arrival"""
        
        # Try to modify the text
        try:

            schemas = list()

            # Get current regex patterns
            patterns = self.get_parameter('replacements').get_parameter_value().string_array_value
            # Parse current regex patterns
            for pattern in patterns:

                # Parse the string
                try:
                    match, replace = str(pattern).split(':=')
                except ValueError as ex:
                    print(f'Pattern "{pattern}" is of invalid form')
                # Keep the pattern pair
                schemas.append({ 'match': match, 'replace': replace})

            # Apply all transformations in the given order
            for schema in schemas:
                msg.data = re.sub(schema['match'], schema['replace'], msg.data)
                
            # Publish the message
            self.pub.publish(msg)

        # On failure, publish the original message
        except:
            self.pub.publish(msg)

# ============================================================== Main ============================================================== #

def main(args=None):

    # Initialize rlc
    rclpy.init(args=args)
    # Create nodes
    minimal_publisher = RegexForwarder()
    # Run nodes
    try:
        rclpy.spin(minimal_publisher)
    except:
        pass
    # Deinitialize rlc
    rclpy.shutdown()

# ================================================================================================================================== #

if __name__ == '__main__':
    main()
