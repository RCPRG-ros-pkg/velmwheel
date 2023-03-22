import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from velmwheel_msgs.msg import EncodersStamped
from tensorflow.data import Dataset

from ament_index_python.packages import get_package_prefix
import numpy as np
import math
from datetime import datetime
import csv
 
def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
 
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
 
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
 
    return roll_x, pitch_y, yaw_z # in radians

class DataCollector(Node):

    def __init__(self):
        super(DataCollector, self).__init__('data_collector')

        self.publisher = self.create_publisher(String, '/test', 10)
        self.timer = self.create_timer(1/30, self.timer_callback)
        self.state = []
        self.state_buffer = []

        self.encoders_subscriber = self.create_subscription(EncodersStamped, '/velmwheel/base/encoders', self.msg_callback_base_encoders, 10,)
        self.encoders_wheel_angles = []
        self.encoders_wheel_angular_velocity = []

        self.odometry_encoders_subscriber = self.create_subscription(Odometry, '/velmwheel/odom/encoders', self.msg_callback_odom_enc, 10,)
        self.odometry_encoders_position = []
        self.odometry_encoders_linear_velocity = []
        self.odometry_encoders_angle = None
        self.odometry_encoders_angular_velocity = None

        self.odometry_laser_subscriber = self.create_subscription(Odometry, '/velmwheel/odom/laser', self.msg_callback_odom_laser, 10,)
        self.odometry_laser_position = []
        self.odometry_laser_linear_velocity = []
        self.odometry_laser_angle = None
        self.odometry_laser_angular_velocity = None

        self.imu_subscriber = self.create_subscription(Imu, '/velmwheel/imu/out', self.msg_callback_imu, 10,)
        self.imu_angle = None
        self.imu_angular_velocity = None
        self.imu_linear_acceleration = []

        self.setpoint = self.create_subscription(Twist, '/velmwheel/base/velocity_setpoint', self.msg_callback_base_vel_setpoint, 10,)
        self.setpoint_angular_velocity = 0.0
        self.setpoint_linear_velocity = [0.0, 0.0]
        print('Hi from gather_data.')

    def timer_callback(self):
        self.state = self.encoders_wheel_angles + self.encoders_wheel_angular_velocity + \
            self.odometry_encoders_position + self.odometry_encoders_linear_velocity + [self.odometry_encoders_angle, self.odometry_encoders_angular_velocity] + \
            self.odometry_laser_position + self.odometry_laser_linear_velocity + [self.odometry_laser_angle, self.odometry_laser_angular_velocity] + \
            [self.imu_angle, self.imu_angular_velocity] + self.imu_linear_acceleration + \
            [self.setpoint_angular_velocity] + self.setpoint_linear_velocity

        if len(self.state) < 27:
            print('State not full.')
            return

        if math.inf in self.state or -math.inf in self.state:
            self.state = [0.0 if substate in [math.inf, -math.inf] else substate for substate  in self.state]
        # print(self.state)

        if not len(self.state_buffer):
            self.state_buffer = [self.state]
        else:
            self.state_buffer.append(self.state)
        # print(len(self.state_buffer))

        if len(self.state_buffer) == 1000:
            dataset = Dataset.from_tensor_slices(self.state_buffer)
            print('Saving dataset')
            # for element in dataset:
            #     print(element)
            #     break
            dataset.save(f'odometry_anomaly_detector/sets/odometry_anomaly_detector_set_{datetime.now()}')
            self.state_buffer = []

    def msg_callback_base_encoders(self, msg):
        self.encoders_wheel_angles = [msg.encoders[i].angle for i in range(4)]
        self.encoders_wheel_angular_velocity = [msg.encoders[i].velocity for i in range(4)]

    def msg_callback_odom_enc(self, msg):
        self.odometry_encoders_position = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        self.odometry_encoders_linear_velocity = [msg.twist.twist.linear.x, msg.twist.twist.linear.y]
        self.odometry_encoders_angle = euler_from_quaternion(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)[2]
        self.odometry_encoders_angular_velocity = msg.twist.twist.angular.z

    def msg_callback_odom_laser(self, msg):
        self.odometry_laser_position = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        self.odometry_laser_linear_velocity = [msg.twist.twist.linear.x, msg.twist.twist.linear.y]
        self.odometry_laser_angle = euler_from_quaternion(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)[2]
        self.odometry_laser_angular_velocity = msg.twist.twist.angular.z

    def msg_callback_imu(self, msg):
        self.imu_angle = euler_from_quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)[2]
        self.imu_angular_velocity = msg.angular_velocity.z
        self.imu_linear_acceleration = [msg.linear_acceleration.x, msg.linear_acceleration.y]

    def msg_callback_base_vel_setpoint(self, msg):
        self.setpoint_angular_velocity = msg.angular.z
        self.setpoint_linear_velocity = [msg.linear.x, msg.linear.y]

def main():
    # Initialize rlc
    rclpy.init()
    # Create nodes
    node = DataCollector()
    # Run nodes
    try:
        rclpy.spin(node)
    except Exception as e:
        print(e)
    # except:
    #     pass

if __name__ == '__main__':
    main()
