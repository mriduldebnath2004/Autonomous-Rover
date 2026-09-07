from setuptools import find_packages, setup

package_name = 'rover_tools'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mriduldebnath',
    maintainer_email='your_email@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        "console_scripts": [
            "keyboard_teleop = rover_tools.keyboard_teleop_node:main",
            "ekf_monitor = rover_tools.ekf_monitor_node:main",'odom_path_publisher = rover_tools.odom_path_publisher:main',
            "rover_stack_manager = rover_tools.rover_stack_manager_node:main",
        ],
    },
)
