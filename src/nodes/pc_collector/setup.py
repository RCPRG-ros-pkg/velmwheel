from setuptools import setup

package_name = 'pc_collector'

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
    description='Package for gathering pointcloud messages',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'collect_pc = pc_collector.collect_pc:main'
        ],
    },
)
