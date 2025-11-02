from setuptools import find_packages, setup

package_name = 'task4_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name+'/launch', ['launch/tb3.launch.py']),
        ('share/' + package_name+'/launch', ['launch/gazebolaunch.py']),
        ('share/' + package_name+'/launch', ['launch/state_publisher.py']),
        ('share/' + package_name+'/config', ['config/botnav.yaml']),
        ('share/' + package_name+'/config', ['config/param.yaml']),
         ('share/' + package_name+'/config', ['config/turtlebot3_lds_2d.lua']),
        ('share/' + package_name+'/urdf', ['urdf/my_bot.urdf']),
        ('share/' + package_name+'/meshes/bases', ['meshes/bases/waffle_base.stl']),
        ('share/' + package_name+'/meshes/sensors', ['meshes/sensors/lds.stl']),
        ('share/' + package_name+'/meshes/sensors', ['meshes/sensors/r200.dae']),
        ('share/' + package_name+'/meshes/wheels', ['meshes/wheels/left_tire.stl']),
        ('share/' + package_name+'/meshes/wheels', ['meshes/wheels/right_tire.stl']),
        ('share/' + package_name+'/maps', ['maps/map.yaml']),
        ('share/' + package_name+'/maps', ['maps/map.pgm']),
        ('share/' + package_name+'/maps', ['maps/map_1760536528.pgm']),
        ('share/' + package_name+'/maps', ['maps/map_1760536528.yaml']),
        ('share/' + package_name+'/Custom_world', ['Custom_world/model.world']),
        ('share/' + package_name+'/Custom_world', ['Custom_world/model.config']),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pranabpandey31',
    maintainer_email='pranabpandey06@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
