# ====================================================================================================================================
# @file       show_map.py
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Thursday, 31st March 2022 1:05:05 pm
# @modified   Thursday, 26th May 2022 12:01:05 am
# @project    engineering-thesis
# @brief      Auxiliary node publishing an RVIZ marker representing an intensity map parsed from the given XML file
#    
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Imports ============================================================ #

# System imports
import sys
# ROS imports
import rclpy
from rclpy.time import Duration
from rclpy.node import Node
# Interfaces imports
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
# External imports
import xml.etree.ElementTree

# ============================================================== Node ============================================================== #

class MapBroadcaster(Node):

    """Auxiliary node publishing an RVIZ marker representing an intensity map parsed from the given XML file"""

    def __init__(self, file):

        """Creates an uxiliary node publishing an RVIZ marker representing an  
        intensity map parsed from the given XML file
        
        Parameters
        ----------
        file : str
            path to the XML file storing the map to be broadcasted
            
        """

        # Initialize node
        super().__init__('poi_map_broadcaster')

        # Create publisher object
        self.publisher = self.create_publisher(Marker, 'poi_map_visualization', 100)
        # Create cyclical time calling a callback each 100ms
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Load the map
        map_root = xml.etree.ElementTree.parse(file).getroot()
        # Create a local visualization message
        self.visualization = Marker()

        # Fill the header of the message
        self.visualization.header.stamp    = self.get_clock().now()
        self.visualization.header.frame_id = "map"
        # Fill the header of the message
        self.visualization.ns           = "visualized_map"
        self.visualization.id           = 1
        self.visualization.type         = Marker.POINTS
        self.visualization.action       = 0
        self.visualization.lifetime     = Duration(seconds=0.5)
        self.visualization.scale.x      = 0.1
        self.visualization.scale.y      = 0.1
        self.visualization.frame_locked = True

        # Creat a color for visualization points
        color = ColorRGBA(
            r = 1,
            g = 0,
            b = 0,
            a = 1
        )

        # Parse points into the visualization
        for i in range(len(map_root.getchildren())):

            # Inser visualization point 
            self.visualization.points.insert(i, Point(float(map_root[i][0].text), float(map_root[i][1].text), 0))
            # Inser co0lour of the point 
            self.visualization.colors.insert(i, ColorRGBA(r = 1, g = 0, b = 0, a = 1))
            
    def timer_callback(self):

        """Publishes marker representing loaded intensity map"""
        
        # Publish the map
        self.publisher.publish(self.visualization)

# ============================================================== Main ============================================================== #

def main():

    # Initialize rlc
    rclpy.init()
    # Create nodes
    broadcaster = MapBroadcaster(sys.argv[1])
    # Run nodes
    rclpy.spin(broadcaster)
    # Deinitialize rlc
    rclpy.shutdown()

# ================================================================================================================================== #

if __name__ == '__main__':
    main()
