import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from time import sleep
from rclpy.signals import SignalHandlerOptions

class RobotController(Node):

    def __init__(self):
        super(RobotController, self).__init__('RobotController')
        self.publisher = self.create_publisher(Twist, '/velmwheel/base/velocity_setpoint', 10)
        self.msg = Twist()
        self.timer = self.create_timer(1/30, self.timer_callback)
        self.time = 0
        self.route = [
        (0.0, 0.0, 0.0, 0.0),
        (5.0, 0.08, 0.0, 0.0),
        (15.0, 0.0, 0.08, 0.0),
        (25.0, 0.0, 0.0, -0.16),
        (35.0, 0.0, 0.0, 0.16),
        (55.0, 0.0, 0.0, -0.16),
        (65.0, -0.08, 0.0, 0.0),
        (75.0, 0.0, -0.08, 0.0),
        (85.0, 0.08, 0.08, 0.0),
        (95.0, -0.08, -0.08, 0.0),
        (105.0, 0.0, 0.08, 0.0),
        (115.0, 0.08, -0.08, 0.0),
        (125.0, -0.08, 0.08, 0.0),
        (135.0, 0.0, -0.08, 0.0),
        (145.0, 0.0, 0.0, 0.0),
        ]

    def timer_callback(self):
        if len(self.route):
            if self.time >= self.route[0][0]:
                self.msg.linear.x = self.route[0][1]
                self.msg.linear.y = self.route[0][2]
                self.msg.angular.z = self.route[0][3]
                self.route.pop(0)
        self.publisher.publish(self.msg)
        self.time += 1/30


def main():
    # Initialize rlc
    rclpy.init(signal_handler_options = SignalHandlerOptions.NO)
    # Create nodes
    node = RobotController()
    # Run nodes
    try:
        rclpy.spin(node)
    except KeyboardInterrupt as e:
        print(e)
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.angular.z = 0.0
        node.publisher.publish(msg)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()