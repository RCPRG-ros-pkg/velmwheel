from setuptools import setup
import os
from glob import glob

package_name = 'odometry_anomaly_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch','*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Daniel Giełdowski',
    maintainer_email='dan.gield@gmail.com',
    description='Anomaly detector in odometry readings',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gather_data = odometry_anomaly_detector.gather_data:main',
            'test_autoencoder_architectures = odometry_anomaly_detector.test_autoencoder_architectures:main',
            'train_and_test_ae = odometry_anomaly_detector.train_and_test_ae:main',
            'corrupt_imu_data = odometry_anomaly_detector.corrupt_imu_data:main',
            'test_ae_on_saved_data = odometry_anomaly_detector.test_ae_on_saved_data:main',
            'perform_random_moves = odometry_anomaly_detector.perform_random_moves:main',
        ],
    },
)
