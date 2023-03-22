from setuptools import setup

package_name = 'odometry_anomaly_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'train_autoencoder = odometry_anomaly_detector.train_autoencoder:main'
        ],
    },
)
