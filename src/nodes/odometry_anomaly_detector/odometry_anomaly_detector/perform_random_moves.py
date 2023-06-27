import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import random
from rclpy.signals import SignalHandlerOptions

class RobotController(Node):

    def __init__(self):
        super(RobotController, self).__init__('robot_controller')
        self.vel_x_max = 0.08338590849833288
        self.vel_x_min = - self.vel_x_max
        self.vel_y_max = self.vel_x_max
        self.vel_y_min = self.vel_x_min
        self.rot_z_max = 0.16677181699666577
        self.rot_z_min = - self.rot_z_max
        self.publisher = self.create_publisher(Twist, '/velmwheel/base/velocity_setpoint', 10)
        self.msg = Twist()
        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        self.msg.linear.x = random.uniform(self.vel_x_min, self.vel_x_max)
        self.msg.linear.y = random.uniform(self.vel_y_min, self.vel_y_max)
        self.msg.angular.z = random.uniform(self.rot_z_min, self.rot_z_max)
        self.publisher.publish(self.msg)

    # def destroy_node(self):
    #     self.msg.linear.x = 0.0
    #     self.msg.linear.y = 0.0
    #     self.msg.angular.z = 0.0
    #     self.publisher.publish(self.msg)
    #     super(RobotController, self).destroy_node()

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