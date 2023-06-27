from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		Node(
			package = 'odometry_anomaly_detector',
			executable = 'gather_data',
			name = 'gather_data',
			),
		Node(
			package = 'odometry_anomaly_detector',
			executable = 'perform_random_moves',
			name = 'perform_random_moves',
			),
		])