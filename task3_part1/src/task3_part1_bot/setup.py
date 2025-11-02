from setuptools import find_packages, setup

package_name = 'task3_part1_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name+'/launch', ['launch/gazebo_launch.py']),
        ('share/' + package_name+'/launch', ['launch/state_publisher.py']),
        ('share/' + package_name+'/urdf', ['urdf/my_bot.urdf']),
        ('share/' + package_name+'/urdf', ['urdf/aruco_bot.sdf']),
        ('share/' + package_name+'/world', ['world/aruco_marker_0.png']),
        ('share/' + package_name+'/world', ['world/world.sdf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pranabpandey31',
    maintainer_email='pranabpandey06@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "aruco_detector=task3_part1_bot.aruco_detector:main",
            "follow=task3_part1_bot.follow:main"
        ],
    },
)
