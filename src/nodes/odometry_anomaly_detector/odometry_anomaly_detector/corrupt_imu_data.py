import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from rclpy.clock import ROSClock
from datetime import datetime, timedelta
import random

class FalseImu(Node):
	"""docstring for FalseImu"""
	def __init__(self):
		super(FalseImu, self).__init__('false_imu')
		self.imu_subscriber = self.create_subscription(Imu, '/velmwheel/imu/out/clear', self.msg_callback_imu, 10,)
		self.imu_publisher = self.create_publisher(Imu, '/velmwheel/imu/out', 10,)
		self.fake_msg = Imu()
		self.declare_parameter('anomaly_type', 'none')
		self.declare_parameter('corrupt_time', 60)
		self.declare_parameter('max_noise', 0.1)
		self.corrupt_time = datetime.now() + timedelta(seconds = self.get_parameter('corrupt_time').get_parameter_value().integer_value)
		# self.fake_msg.linear_acceleration.z = 9.8
		self.lagged_message = None

	def msg_callback_imu(self, msg):
		if datetime.now() > self.corrupt_time:
			self.imu_publisher.publish(self.generate_anomaly(msg))
		else:
			self.imu_publisher.publish(msg)

	def generate_anomaly(self, msg):
		anomaly = self.get_parameter('anomaly_type').get_parameter_value().string_value
		# print(anomaly)
		if anomaly == 'none':
			if self.lagged_message:
				self.lagged_message = None
			self.fake_msg = msg
		elif anomaly == 'blackout':
			if self.lagged_message:
				self.lagged_message = None
			self.fake_msg = Imu()
			self.fake_msg.header = msg.header
		elif anomaly == 'lag':
			if self.lagged_message is None:
				self.lagged_message = msg
			self.fake_msg = self.lagged_message
			self.fake_msg.header = msg.header
		elif anomaly == 'noise':
			if self.lagged_message:
				self.lagged_message = None
			self.fake_msg = msg
			noise_val = self.get_parameter('max_noise').get_parameter_value().double_value
			self.fake_msg.linear_acceleration.x = self.fake_msg.linear_acceleration.x + random.uniform(-noise_val, noise_val)
			self.fake_msg.linear_acceleration.y = self.fake_msg.linear_acceleration.y + random.uniform(-noise_val, noise_val)
			self.fake_msg.angular_velocity.z = self.fake_msg.angular_velocity.z + random.uniform(-noise_val, noise_val)
		return self.fake_msg

def main():
    # Initialize rlc
    rclpy.init()
    # Create nodes
    node = FalseImu()
    # Run nodes
    try:
        rclpy.spin(node)
    except Exception as e:
        print(e)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
	main()