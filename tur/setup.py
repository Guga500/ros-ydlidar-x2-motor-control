from setuptools import setup

package_name = 'gp650_turtlesim_teleop'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/teleop.launch.py']),
        ('share/' + package_name + '/config', ['config/gp650_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Teleop turtlesim with GP-650 gamepad',
    license='MIT',
    entry_points={
        'console_scripts': [
            'teleop = gp650_turtlesim_teleop.teleop_node:main',
        ],
    },
)

