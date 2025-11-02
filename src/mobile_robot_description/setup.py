from setuptools import find_packages, setup

package_name = 'mobile_robot_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/nav2_simulation.launch.py']),
        ('share/' + package_name + '/config', ['config/nav2_params.yaml']),
        ('share/' + package_name + '/rviz', ['rviz/nav2_default_view.rviz']),
        ('share/' + package_name + '/maps', ['maps/map.yaml', 'maps/map.pgm']),
        ('share/' + package_name + '/urdf', ['urdf/robot.urdf.xacro']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alan',
    maintainer_email='przemekcinkowski1@gmail.com',
    description='Mobile robot description and Nav2 integration for forklift robot',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'velocity_monitor = mobile_robot_description.velocity_monitor:main',
        ],
    },
)
